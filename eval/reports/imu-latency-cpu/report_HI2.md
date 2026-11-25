# IMU/Image 延迟 × CPU 负载（imu_latency_test.py 封装）

- 生成时间：2026-11-25T17:20:36
- 阶段：空载 60.0s → 压力 60.0s（3 核） → 恢复 60.0s
- image stamp 域：`mono`；IMU 域：`realtime`
- **启动窗口剔除**：前 5s（IMU 1524 / Image 0 样本）——IIO 缓冲积压与订阅建立属启动瞬态，不参与稳态判定
- **偏移剥离**：frame_in 的 `ts_mono−ts_frame` 中位数 **22.019ms**（boot 相关；表中 image 值已减去该偏移）

## 判定：**PASS**

| 检查项 | 结果 | 明细 |
|---|---|---|
| IMU 空载 p99 ≤ 5.0ms（/imu_data） | ✅ | p99=3.975ms |

## IMU 延迟（recv_rt − stamp，realtime 域）

| topic | 阶段 | 样本 | 均值(ms) | p50 | p95 | p99 | max |
|---|---|---|---|---|---|---|---|
| /imu_data | 空载 | 11131 | 3.14 | 3.06 | 3.70 | 3.98 | 12.71 |
| /imu_data | 压力 | 12142 | 3.25 | 3.18 | 3.87 | 4.15 | 10.30 |
| /imu_data | 恢复 | 12041 | 3.14 | 3.08 | 3.70 | 3.96 | 4.94 |

## 口径与局限

- 测量端即 `imu_latency_test.py`（Python 单线程订阅）；压力期其自身也被挤占，延迟上升含「测量端被饿」成分。
- 阶段边界按「首样本 + pre_stress」推算（脚本 CSV 无阶段标记），误差 <1s。
- image 绝对值受 boot 相关偏移影响；已剥离时可比，未剥离时仅同 run 相对比较。
- IMU 订阅用默认 reliable depth=1000，与产品 best_effort 消费者行为不同。
