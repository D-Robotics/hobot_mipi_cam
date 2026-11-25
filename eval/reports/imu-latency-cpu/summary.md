# IMU/Image 延迟 × CPU 负载（imu_latency_test.py 接入框架）

| 项 | 内容 |
|---|---|
| 日期 | 2026-11-25（板端重刷新系统后首轮） |
| 脚本 | 仓库根 `imu_latency_test.py`（**零修改**，仅由框架传 CLI） |
| 用例 | HI1：IMU+图像（180s：空载 60 / 压力 60 / 恢复 60）<br>HI2：**只测 IMU**（同配置，不订图像）——对照，用于定位测量污染 |
| 平台 | RDK X5（8 核 A55），sc132gs 双目 + icm42688，root 运行 |
| 原始数据 | `eval/logs/HI1_..._170810/`、`HI2_..._171514/` |
| 指标 | `metrics_HI1.json`、`metrics_HI2.json`、`report_HI{1,2}.md` |

## 1. 结论

| 指标 | 实测 | 判定 |
|---|---|---|
| **IMU 采样→接收延迟（稳态）** | p50 **3.06ms**、p95 3.70、p99 **3.98**、max 12.71 | ✅ |
| **IMU 延迟受 3 核 CPU 压力影响** | 空载 3.06 → 压力 **3.18** → 恢复 3.08（+3.9%） | ✅ 几乎无影响 |
| **图像延迟（剥离 boot 偏移后）** | 左 20.48ms / 右 27.19ms（p50） | 记录 |
| **图像延迟受压力影响** | 左 p95 30.66 → 33.26（**1.08×**）→ 32.61（1.06×）<br>右 33.60 → 33.89（1.01×）→ 34.59（1.03×） | ✅ ≤2× |
| HI1 中的 IMU 延迟 | p50 空载 **1526ms**、压力 56ms、恢复 1934ms | ❌ **不可用**（见 §2③） |

**可复现性**：用 `run_suite.py --suite imu-latency` 端到端复跑一次，HI2 的 IMU 稳态为
p50 3.02 / 压力 3.21 / 恢复 3.09ms（p99 3.93/4.20/4.03），与首轮一致（差异 <2%）。

## 2. 三种必须处理的测量口径（本轮实测坐实）

**① 启动瞬态**：IMU 前 5s 延迟可达 7s（IIO 缓冲积压 + 订阅建立）。剔除前 5s 后：
p99 5702ms → **3.98ms**，5/10/20/30s 剔除结果一致 ⇒ 瞬态仅在开头。
分析器默认 `--warmup-s 5`，报告显式记录剔除样本数。

**② image stamp 的 boot 相关偏移**：本次 `ts_mono−ts_frame` = **20.1 / 22.0ms**（历次 37.8ms、116.6ms、3.9s）。
image 延迟 = `recv_mono − stamp` 含此偏移，分析器已剥离；不剥离时**只能同 run 相对比较**。

**③ 脚本自身架构对 IMU 的污染（本轮最大发现）**：
HI1 与 HI2 唯一差别是「是否订阅图像」，IMU 延迟从 **3.06ms 变成 1526ms**（500 倍）。

| | HI1（IMU+图像） | HI2（仅 IMU） |
|---|---|---|
| IMU 空载 p50 | 1526.5ms | **3.06ms** |
| IMU 压力 p50 | 56.5ms | 3.18ms |
| IMU 恢复 p50 | 1933.5ms | 3.08ms |

机制：脚本 `SingleThreadedExecutor` 中图像回调要反序列化 2.09MB（Python），期间 IMU 消息在
**reliable depth=1000** 队列积压（最多 5s）；压力期图像回调变少、积压反而下降 → 出现
「压力期延迟更低」的反向现象。**HI1 的 IMU 数值不反映产品，HI2 才是 IMU 真实延迟。**

## 3. 与已有评测链的对照

| 链路 | 数值 | 来源 |
|---|---|---|
| IMU 采样→Python 回调 | **3.06ms**（含驱动读+发布+DDS+反序列化） | 本轮 HI2 |
| 图像 pub→recv（DDS 段） | 1.2~2.1ms | 16 轮（C++ recorder） |
| 图像就绪→发布（设备侧） | 10.6ms（左）/15.5ms（combine） | 08 轮 |
| 图像 t0→recv | 14.1ms（lpwm 可信 t0） | 16 轮 |
| 本轮图像 recv−stamp（剥偏移） | 20.5/27.2ms | HI1（含 Python 反序列化，比 C++ 高 ~5~10ms） |

## 4. 框架接入（不改脚本）

| 文件 | 作用 |
|---|---|
| `eval/cases/HI1.yaml` / `HI2.yaml` | 用例：C04 配置 + 三阶段 60/60/60 + 图像订阅开关 + 阈值 |
| `eval/run/run_latency.sh` | 起 launch（开 `[EVAL] exposure,frame_in`）→ 等首帧 → 调脚本 → `result.txt` 判绿 |
| `eval/tools/analyze/latency_csv.py` | 三阶段分段统计 + 偏移剥离 + 启动剔除 + 三态判定 + 报告 |
| `thresholds.py` | `imu_latency_p99_ms=5`、`image_stress_p95_ratio=2`、`image_post_p95_ratio=1.2` |

- 不起 recorder（脚本即订阅端），避免第二个订阅者抢 CPU；
- 压力完全由脚本内部 `--stress_cores` 管（本轮 3/8 核）；
- 阶段边界按「首样本 + pre_stress」推算（CSV 无阶段标记），误差 <1s。

一键跑（含三态汇总）：

```bash
ssh s600 'cd /home/sunrise/workspace/hobot_mipi_cam && ./eval/run/run_suite.py --suite imu-latency'
# → eval/logs/suite_imu-latency_<ts>/suite_report.md（HI1 WARN / HI2 PASS）
```

## 5. 对脚本的改进建议（未实施——按你的要求不改脚本）

1. **IMU 与图像分线程/分节点订阅**（或 IMU 用 best_effort），否则 IMU 延迟永远被图像反序列化污染；
2. 汇总表按 pre/stress/post 分段输出（现在只有全程混合值）；
3. 启动窗口剔除（或先丢弃 5s 再开始计时）；
4. 记录 `image_domain` 与 `ts_mono−ts_frame` 偏移，便于剥离。

## 6. 局限

- 压力 3/8 核（保守）；更重的负载（如 7 核）未测。
- 重刷后首轮，未做多轮复现；image 空载曾出现一次 157ms 尖峰（best_effort 丢样/调度）。
- IMU reliable depth=1000 与发布端 depth=10 的背压关系未单独量化。
