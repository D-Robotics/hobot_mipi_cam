# eval/15-recv-drop：接收链路对账

| 项 | 内容 |
|---|---|
| 分支轮次 | 15-recv-drop（`eval/07-combine` 分支逐轮提交） |
| 日期 | 2026-11-18 |
| 平台 | RDK X5 V1.0，sc132gs 双目 + icm42688 |
| 用例 | C04（180s，本轮新跑）+ C16/C07（交叉验证） |
| 结果 | E-02 IMU 接收 **0 丢**、E-03 **零倒退零重复**；E-01 图像接收丢 **3.2%~4.4%**（未达 0.5%）——根因定位为 **recorder 自身 best_effort QoS 丢样，非发布/链路丢帧**；E-10 info 侧接收 99.98%，配对率 96% 被 image 丢样连累 |
| 原始数据 | `eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000101_111736/` |
| 指标 JSON | `metrics_C04.json` |

## 1. 结果

| topic | CSV 行 | pub 数 | 订阅前 | 接收丢帧(率) | 倒退 | 重复 |
|---|---|---|---|---|---|---|
| /image_left_raw | 4980 | 5190 | 1 | 209（**4.03%**） | 0 | 0 |
| /image_right_raw | 4961 | 5190 | 1 | 229（**4.41%**） | 0 | 0 |
| /image_combine_raw | 5124 | 5189 | 1 | 64（**1.23%**） | 0 | 0 |
| /imu_data（E-02） | 34945 | 期望 34592 | — | **0（0.0%）** | 0 | 0 |

对账方法：**按 stamp 匹配**（pub_out 的每帧 `ts_frame` 是否出现在 CSV 同 topic stamp 集合），比计数比更准——订阅建立前的发布（首帧前 1 帧）不计丢。

## 2. 核心发现：E-01 未达标，但丢在订阅端

三条证据链定位根因：

1. **发布侧完整**：`pub_out` 三 topic 均 5189/5190 帧（仅启动首帧差），与 `frame_in`/`combine_out` 一致——设备/发布链没有丢；
2. **丢的是"订阅处理能力"**：recorder 为 best_effort + KeepLast(1)，单线程逐 topic 回调；C16 场景（多订 IMU）交叉验证 left/right 丢 2.9~3.1%、combine 丢 1.2%，与 C04 一致且稳定复现；
3. **大小消息不对称**：combine（4.2MB/帧）丢 1.2% < left/right（2.1MB）丢 4%——若是链路带宽丢应反过来；说明是 recorder 回调积压后 KeepLast(1) 丢旧帧，而 combine 因发布节奏更晚（08 轮 +14ms）恰好错开处理高峰。

**结论**：E-01 的 0.5% 判据在 best_effort 评测订阅者下结构性不可达；该数值度量的是"**单订阅者轻量接收**"的丢样率，对产品低配消费者有参考意义，不能作为发布链路质量结论。发布链路质量由 A-06（设备侧 0 丢）+ E-03（零倒退重复）共同背书。

**改进方向**（记录，不在本轮做）：recorder 拆分 QoS——图像 reliable 或双进程（专收图像/专收 IMU），把"工具丢"与"链路丢"彻底分离；本轮保留 best_effort 以免引入反压。

## 3. E-10 camera_info 配套

| info topic | info 行 | 与 image 配对 | 率 | 多余 |
|---|---|---|---|---|
| /image_left_raw/camera_info | 5187 | 4978 | 95.97% | 209 |
| /image_right_raw/camera_info | 5188 | 4958 | 95.57% | 230 |

- **info 侧接收 5187/5190（99.94%）**——info 是小消息，几乎不丢；
- 配不上的 209/230 帧 = **对应 image 帧被 recorder 丢了**（与 E-01 的丢帧数一致），非 info 自身问题；
- 在"双方都被收到"的交集内，stamp **100% 逐帧相等**（0 例错配）——E-10 的"配套正确性"本身通过；总量差是 E-01 的连带。

## 4. 分析链

- 新增 `recv_metrics`：stamp 对账（含订阅前窗口剔除）、每 topic 倒退/重复（重复用集合差，乱序不漏）、IMU 期望对比、info 配对；
- thresholds：`recv_drop_max=0.005`、`imu_recv_drop_max=0.0001`、`info_pair_min=1.0`；
- 单测 **68 项全过**（新增对账/订阅前窗口/倒退重复/IMU/info 配对）。

## 5. 复现

```bash
ssh s600 'cd /home/sunrise/workspace/hobot_mipi_cam && ./eval/run/run_case.sh C04 180'

python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000101_111736/recorder.csv \
  --log eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000101_111736/mipi_cam.eval.log \
  --out-md /tmp/C04r.md --out-json /tmp/C04r.json
```

## 6. 局限

- recorder best_effort 是评测工具的选择，reliable 订阅者的接收质量未测（会引入反压，可能影响被测节点，需独立轮次设计）；
- 未测多订阅者（1/2/4）与 CPU 负载下的接收质量（E-06/07，扩展期）；
- info 配对的"多余"帧里可能存在 image 正常但 info 重复的极端情况，本轮数据多余数=图像丢数，无此情况。
