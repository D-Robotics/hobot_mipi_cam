# eval/03-drop：设备侧采集/队列丢帧 + 正向对照

| 项 | 内容 |
|---|---|
| 分支轮次 | `eval/03-drop` |
| 日期 | 2026-11-12 |
| 平台 | RDK X5 V1.0，kernel 6.1.83，TROS humble |
| 传感器 | sc132gs 双目（pipe0/pipe1） |
| 正常用例 | C04：双目 1088x1280@30 nv12 / `dual_combine=1`，180s |
| 正向对照 | C04S：同配置，node 与 2×`yes` 钉在 CPU0，60s |
| 结果 | 两轮均 `RESULT=OK` |
| 原始数据 | 正常：`eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_084221/`<br>压力：`eval/logs/C04S_dual-1088x1280-nv12-30fps-combine1-stress_20000104_084604/` |
| 图表 | `plots/drop_by_reason_stress.png` |
| 指标 JSON | `metrics.json`（含 normal/stress 两份） |

## 1. 正常 C04：0 丢帧

| 流 | 收到帧 | 丢帧总数 | 丢帧率 | queue_recycle | empty_null | fq_full | fq_expired | getframe_fail | frame_id缺口(归因) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `pipe0/main` | 5186 | 0 | 0% | 0 | 0 | 0 | 0 | 0 | 0 |
| `pipe1/main` | 5185 | 0 | 0% | 0 | 0 | 0 | 0 | 0 | 0 |

- `queue_stat` 共 346 行（2 pipe × 1Hz × 172.8s），说明计数器线程正常工作，**0 是实测 0，不是缺数据**。
- `frame_drop` 事件 0 条；`select_timeout/select_error/getframe_fail` 均 0。
- 帧时序：30.001 / 30.000 fps，间隔 p99 33.354 / 33.361ms（与 02 轮一致）。

## 2. C04S 正向对照：计数器能抓到丢帧

| 流 | 收到帧 | 丢帧总数 | 丢帧率 | queue_recycle | fq_expired | getframe_fail | frame_id缺口(归因) |
|---|---:|---:|---:|---:|---:|---:|---:|
| `pipe0/main` | 1419 | 568 | 28.59% | 476 | 92 | 0 | 44 |
| `pipe1/main` | 1343 | 415 | 23.61% | 399 | 16 | 0 | 194 |

- combine 队列：`recycle=803`，`empty_null=0`，depth=3。
- `empty_null=0`、`fq_full=0`、`getframe_fail=0`、`task_drops={}`；丢帧集中在 **queue_recycle**（发布侧跟不上，最旧帧被回收）和 **fq_expired**（拼接配对丢过期帧）。
- 压力下设备侧采集仍约 29.5 / 28.0 fps，但接收端只有 ~19.6 / 18.8 fps——与“设备侧 28.6% / 23.6% 帧被丢”一致。
- 结论：**正常 0 + 压力 >0，证明丢弃点计数器有效**，不是“永远为 0 的假计数器”。

## 3. 直接证据与关键修正

- 打点位置：
  - `queue_recycle`：`BuffQueueManage::get_empty_buff()` 在 `q_buff_.size()>=3` 回收队头时 `fetch_add(1)`；
  - `empty_null`：`get_empty_buff()` 返回空时计数；
  - `fq_full` / `fq_expired`：`FrameQueue::push` 满丢 / `popUntil` 过期丢；
  - `getframe_fail` / `select_timeout` / `select_error`：X5 主/辅采集循环逐次 `frame_drop` 事件；
  - `queue_stat`：1Hz 线程输出各队列累计计数与水位。
- **`frame_id` 步长修正**：X5 的 `frame_id` 按 sensor 帧递增（C04 实测步长 **2**，sensor 60fps → VSE 30fps）。直接算 `max−min+1−unique` 会把每帧都算成缺口（normal 会误报 5185）。现改为按**最常见步长**计算超步缺口：normal 0，stress 44/194。该值仅作 VIN 侧归因，不计入主丢帧率。
- 主丢帧率只使用直接丢弃点计数，不使用 `frame_id` 反推。

## 4. 本轮顺带观察（不归本轮结论）

| 现象 | 证据 | 归属 |
|---|---|---|
| 接收端比 `pub_out` 少约 1.8% | 正常左 5183→5091、右 5184→5098、拼接 5183→5137 | `eval/15-recv-drop` |
| 压力下接收端 ~19.6fps | 设备侧丢 28.6%，接收端随之下降 | 正向对照预期，不是产品基线 |
| 关机 `grab failed`/`hb_mem_free_buf` | 日志末尾 | `eval/20-start-stop` |
| `/imu_data` 无数据 | `/dev/iio:device1` root-only | IMU 分支 |

## 5. 评审

- 正常 C04：`result.txt`、节点/recorder 日志、`drop` 统计均无异常（0 丢帧、计数器在）。
- C04S：丢帧是**正向对照的预期结果**，原因分解与设备/接收帧率一致。
- 按约定**不启动子代理评审**：log/analysis clean, no review。
- 本轮自行发现并修复了 `frame_id` 步长导致的归因误报（见 §3）。

## 6. 复现

```bash
# 板端（192.168.127.10，NFS 仓库）
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_case.sh C04 180      # 正常
./eval/run/run_case.sh C04S 60      # 正向对照（CPU0 + 2×yes）

# 主机分析
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_084221/recorder.csv \
  --log eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_084221/mipi_cam.eval.log \
  --out-md /tmp/C04_drop_normal.md --out-json /tmp/C04_drop_normal.json
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C04S_dual-1088x1280-nv12-30fps-combine1-stress_20000104_084604/recorder.csv \
  --log eval/logs/C04S_dual-1088x1280-nv12-30fps-combine1-stress_20000104_084604/mipi_cam.eval.log \
  --out-md /tmp/C04_drop_stress.md --out-json /tmp/C04_drop_stress.json
```

## 7. 局限

- 正向对照使用 `yes`（板端无 `stress-ng`）+ CPU0 亲和，目的是证明计数器有效，**不是产品负载基线**。
- 仅测 `dual_combine=1` 主码流；`sub_stream`、`single`、hbmem 通路未在本轮覆盖。
- `queue_stat` 为累计计数，1Hz 输出；分析取最后一个周期值。
- `frame_id` 缺口仅归因，不计入主丢帧率；原始序列步长会随 sensor/VSE 分频变化。
