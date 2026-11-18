# eval/13-cam-imu-basic：Camera-IMU 域归一与配对

| 项 | 内容 |
|---|---|
| 分支轮次 | 13-cam-imu-basic（`eval/07-combine` 分支逐轮提交；12-imu-drop 已被 10/11 轮的双口径丢样覆盖，跳过） |
| 日期 | 2026-11-18 |
| 平台 | RDK X5 V1.0，sc132gs 双目 + icm42688（IIO，root 运行） |
| 用例 | C07：双目 1088x1280@30 combine=1 + **lpwm=on**（可信 t0）+ IMU 同录 180s |
| 结果 | D-01 域归一链路验证通过；D-02 配对偏差 p50=**1239µs**（相位随机理论值 1250µs）；D-04 覆盖率 **1.008** |
| 原始数据 | `eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000101_083935/` |
| 指标 JSON | `metrics_C07.json` |

## 1. 结论

| 指标 | 实测 | 参考/判据 | 判定 |
|---|---|---|---|
| D-02 最近邻偏差 p50 | **1239µs** | 相位随机理论 T/4=1250µs | ✅ 无系统偏差 |
| D-02 p99 / max | 2459µs / 70.2ms | p99 阈值 10ms | ✅ / max 为 IMU 缺口帧 |
| D-04 覆盖率 | **1.0079**（34884/34612） | ≥0.95 | ✅ 超 100% 因 IMU 实际 202Hz |

- **核心结论**：归一后相机曝光中点与 IMU 采样网格的相对相位呈**随机均匀分布**（p50 与 T/4 理论值重合到 11µs），即**两级域归一（clock 旁车 + trig 硬件域 offset）都没有引入系统性偏差**——05/08 记录的 ±8ms/±1.5ms 不确定度表现为 run 间波动，中位数估计在本 run 内是准的。
- 05 遗留的"3.9s stamp 落后"与 09（已并轮）的域审计问题由此闭环：**图像 t0 与 IMU 在归一后可对齐到采样网格极限**。
- max=70.2ms 出现在 IMU 批量缺口（10/11 轮的内核侧 gap）期间的帧，属 IMU 侧已知问题，非配对链路缺陷。

## 2. 方法（本轮零新埋点）

```
相机侧：exposure(trig_valid) → t0_hw = ts_trig + exp/2
        t0_mono = t0_hw − median(ts_hw − 打点ts_mono)   （08 轮 offset 法）
IMU 侧：stamp(realtime) − clock 旁车偏移 → mono
配对  ：每帧 t0_mono 在 IMU 网格上找最近样本；相邻帧窗口数样本算覆盖率
```

- C07 用例 `log_events` 增加 `imu_probe/imu_stat`（与图像 trig 同录）；分析链新增 `cam_imu_metrics`。
- thresholds：`cam_imu_nearest_p99_ms=10`、`cam_imu_cover_min=0.95`。
- 单测 **64 项全过**（新增合成网格配对/覆盖率、缺席）。

## 3. 局限

- 精度受限于两级归一的中位数估计（run 内准、run 间波动）；亚毫秒级融合需求仍需驱动侧统一时戳域。
- lpwm 开启的配置才可信 t0；无触发场景只能用帧 ts 近似（差半曝光）。
- max 帧的 70ms 缺口归 IMU 内核侧丢样（0.11%~0.28%），未在本轮处理。
- 30fps/200Hz 单一组合；其它帧率组合的覆盖率未测。

## 4. 复现

```bash
ssh s600 'cd /home/sunrise/workspace/hobot_mipi_cam && ./eval/run/run_case.sh C07 180'

python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000101_083935/recorder.csv \
  --log eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000101_083935/mipi_cam.eval.log \
  --out-md /tmp/C07r.md --out-json /tmp/C07r.json
```
