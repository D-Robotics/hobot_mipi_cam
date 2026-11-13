# eval/02-frame-timing：设备侧帧率 / 间隔稳定性 / 时间戳单调与重复

| 项 | 内容 |
|---|---|
| 分支轮次 | `eval/02-frame-timing` |
| 日期 | 2026-11-12 |
| 平台 | RDK X5 V1.0，kernel 6.1.83，TROS humble |
| 传感器 | sc132gs 双目（pipe0/pipe1） |
| 用例 | C04：双目 1088x1280@30 nv12 / ros / sensor ts / `dual_combine=1` |
| 时长 | 180s（有效覆盖 172.78s） |
| 结果 | `RESULT=OK` |
| 原始数据 | `eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_081403/` |
| 图表 | `plots/frame_interval_hist.png` |
| 指标 JSON | `metrics.json` |

## 1. 结果

| 流 | 帧数 | 均值 fps | 间隔 p50 | p95 | p99 | max | 时间戳回退 | 重复 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `pipe0/main` | 5186 | 30.001 | 33.33ms | 33.35ms | 33.35ms | 33.38ms | 0 | 0 |
| `pipe1/main` | 5185 | 30.000 | 33.33ms | 33.35ms | 33.36ms | 33.44ms | 0 | 0 |

- 目标周期 33.333ms；p99 偏差约 0.02–0.03ms（0.06%–0.09%），远低于初始期望 5%（1.67ms）。
- 两路都稳定在 30.0fps；单调性 100%，重复率 0%。
- 日志开启后帧率/间隔与上一轮一致，未观察到 `frame_in` 逐帧打印造成的性能劣化。

## 2. 直接证据

- X5 主码流 `multiFrameTask`、辅码流 `subMultiFrameTask` 在 `getVnodeFrame` 成功（含 memcpy/releaseframe）后逐帧打点：
  ```
  [EVAL] ts_mono=<自动> event=frame_in pipe=<0|1> stream=main ts_frame=<int64 ns>
  ```
- 本轮只加 `pipe / stream / ts_frame`；**未加** `frame_id`（下轮丢帧）、`width/height`（分辨率轮）、`exp_us/ts_trig`（曝光轮）。
- 1 个 pipe 约 30 行/s；双目约 60 行/s，3 分钟 `frame_in` 共 10,371 行，日志量可控。

## 3. 上下文观察（不归本轮结论）

| 现象 | 证据 | 归属 |
|---|---|---|
| 设备侧 `pub_out` 比 `frame_in` 少 1–3 帧 | `frame_in` 5186/5185；`pub_out` 5183/5184 | 启动/收尾窗口，非帧时序问题 |
| 接收端比 `pub_out` 少约 1.3%–1.8% | 左 5183→5089、右 5184→5090、拼接 5183→5146 | `eval/15-recv-drop` |
| `frame_ts` 口径端到端延迟约 3.66s | 报告 `latency.frame_ts` p50≈3659ms | `eval/09-domain-audit`（域未审计前不可作为基线） |
| 关机 3 次 `grab failed` + 2 次 `hb_mem_free_buf` | 日志末尾 | `eval/20-start-stop` |
| `/imu_data` 无数据 | `/dev/iio:device1` root-only；`imu_error` 可见 | IMU 分支 |

## 4. 评审

- 本轮 `result.txt`、节点日志、recorder 日志与 `frame_timing` 统计均无异常（两 pipe 齐全、30fps、无回退/重复）。
- 按约定**不启动子代理评审**：log/analysis clean, no review。
- 已知的关机 `hb_mem_free_buf`、IMU 权限、左右 3ms 相位差与本轮指标无关，已归到对应后续分支。

## 5. 复现

```bash
# 板端（192.168.127.10，NFS 仓库）
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_case.sh C04 180

# 主机分析
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_081403/recorder.csv \
  --log eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_081403/mipi_cam.eval.log \
  --out-md /tmp/C04_report.md --out-json /tmp/C04_metrics.json
python3 eval/tools/analyze/plot_frame_timing.py \
  --log eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_081403/mipi_cam.eval.log \
  --out eval/reports/02-frame-timing/plots/frame_interval_hist.png
```

## 6. 局限

- 仅测 `dual_combine=1` 主码流；`sub_stream`、`single`、`gdc_enable=false`、hbmem 通路未在本轮覆盖。
- `ts_frame` 为 `frame_ts_type=sensor` 的内核帧时间戳；realtime 模式另测。
- 逐帧日志只用于评测；默认 `eval_log_enable=false`，产品行为不变。
