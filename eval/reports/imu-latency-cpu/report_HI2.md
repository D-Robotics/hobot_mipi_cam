# IMU/Image 延迟 × CPU 负载（imu_latency_test.py 封装）

- 生成时间：2026-11-26T10:56:14
- 阶段：空载 60.0s → 压力 60.0s（3 核） → 恢复 60.0s
- image stamp 域：`mono`；IMU 域：`realtime`
- **启动窗口剔除**：前 15s（IMU 0 / Image 0 样本）——IIO 缓冲积压与订阅建立属启动瞬态，不参与稳态判定
- **偏移剥离**：frame_in 的 `ts_mono−ts_frame` 中位数 **373.117ms**（boot 相关；表中 image 值已减去该偏移）

## 判定：**PASS**

| 检查项 | 结果 | 明细 |
|---|---|---|
| IMU 空载 p99 ≤ 5.0ms（/imu_data） | ✅ | p99=4.064ms |

## IMU 延迟（recv_rt − stamp，realtime 域）

| topic | 阶段 | 样本 | 均值(ms) | p50 | p95 | p99 | max |
|---|---|---|---|---|---|---|---|
| /imu_data | 空载 | 12144 | 3.27 | 3.05 | 3.69 | 4.06 | 94.42 |
| /imu_data | 压力 | 12158 | 3.48 | 3.18 | 3.88 | 4.28 | 99.30 |
| /imu_data | 恢复 | 10461 | 3.25 | 3.04 | 3.65 | 4.03 | 100.45 |

## 口径与局限

- 测量端即 `imu_latency_test.py`（IMU/图像分节点分线程订阅）；压力期测量端自身也会被挤占，延迟含部分「测量端被饿」成分。
- 阶段边界取自脚本 `<stem>.phases.json`（精确，非推算）。
- 脚本域检查（`recv−stamp` 最小值，最接近纯偏移）：/imu_data=2.71ms
- image 绝对值受 boot 相关偏移影响；已剥离时可比，未剥离时仅同 run 相对比较。
- IMU 订阅为 best_effort depth=50（不随订阅端变慢积压），与产品 best_effort 消费者行为一致。
