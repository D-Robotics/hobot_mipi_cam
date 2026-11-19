# X5 hobot_mipi_cam 评测总报告

| 项 | 内容 |
|---|---|
| 对象 | `hobot_mipi_cam` v2.5.6 X5 后端（vflow：VIN/ISP/VSE/GDC） |
| 硬件 | RDK X5 V1.0（kernel 6.1.83，TROS humble）+ sc132gs 双目 + icm42688 |
| 周期 | 2026-11-12 ~ 11-19（foundation + 01~20 轮 + 8h 长稳） |
| 方法 | 零侵入 `[EVAL]` 事件埋点（9 类）+ 独立 recorder + 离线分析链（72 项单测）；直接证据优先，每轮独立评审 |
| 交付 | 20+ 份轮次报告（`eval/reports/`）、可复用评测资产（`run_suite.py` 等）、逐轮提交 `e8c9721..HEAD`（eval/07-combine 分支） |

## 1. 总结论

**被测版本在 sc132gs 双目满配下功能与稳定性达标**：采集/同步/延迟/IMU/接收五个计分模块全部有据可判，门槛项（资源/启停）通过，8h 长稳全绿。发现 **1 个确定性产品缺陷**（realtime 时间戳 int32 溢出）、若干特性限制（bgr8+combine 不可用、双目相位随机两档、IMU 权限与内核侧丢样）——均不阻塞 nv12 主路径。

## 2. 分模块指标总览

### A 采集（22 分项）✅

| 指标 | 实测 | 期望 |
|---|---|---|
| A-01 帧率 | 主码流 **30.000** 恒定（3min~8h 全部 run）；辅流 14.0 | ≥99% |
| A-02 间隔 | p99 33.35ms（±0.05%）、max 35ms（8h） | p99≤5% 周期 |
| A-03/04 单调/重复 | 全部 run **0/0** | 100%/0 |
| A-05 首帧 | 5.03s（GDC 2.58s 占 51%） | ≤2s **未达**（GDC 生成属一次性初始化，见 §4-P9） |
| A-06 丢帧 | 常态 **0**；CPU0 满载对照 23~29% | ≤0.5%（常态✅） |
| A-07 分辨率 | 设备/发布/info 三面 **100% 一致**（含 r90、combine） | 100% |

### B 时间戳与同步（28 分项）✅（2 项特性限制）

| 指标 | 实测 | 说明 |
|---|---|---|
| B-01 抖动 | stamp 序列 p99 <0.1ms（30.000fps 恒定佐证） | ✅ |
| B-02 双目同步 | 配对 stamp 差 2.96~3.16ms；**曝光起点相位 ≈1.7ms**；lpwm 档 1.28ms（=左右 AE 曝光时长差） | >1ms 飞书口径：**触发级为 0**，stamp 级不达（见 P6） |
| B-03 配对率 | 99.98~100%，**零错配** | ✅ |
| B-05 拼接 ts | `ts_combine==ts_left` 100%、几何/顺序 100% | ✅ |
| B-06/07/08 延迟 | 就绪→发布 10.6ms（左）/15.5ms（combine）；t0→发布 13.3ms | 基线已立 ✅ |
| B-11 拼接额外 | +14ms（等待另一路 + 10ms 轮询 + 双拷贝） | ✅ 基线 |

### C IMU（11 分项）✅（需 root）

| 指标 | 实测 |
|---|---|
| C-01 频率 | **202.14Hz**（+1.07% ODR 晶振偏差） |
| C-02 抖动 | p99 ±1.1% |
| C-03 单调/重复 | **0/0** |
| C-04 丢样 | 0.11~0.28%（两处 39.5ms 内核侧批量缺口）→ 期望 0.01% **未达**（驱动侧，P7） |
| C-05 有效性 | NaN/Inf=0、模长 **9.786 m/s²**、gyro 零偏 0.00435 rad/s（7 倍余量） |
| C-06 零偏基线 | accel(−0.22, 9.77, 0.49)、gyro(−0.001,−0.004,0.004)；噪声 gyro σ≈0.0005 |

### D Camera-IMU（17 分项）✅

| 指标 | 实测 |
|---|---|
| D-01/02 域归一配对 | nearest p50 **1239µs ≈ 理论 1250µs**（无系统偏差）；p99 2451µs |
| D-04 覆盖率 | **1.008** |
| D-03 漂移 | 30min 斜率 **−0.1175µs/min**（全程 −3.5µs）、零跳变——无可测漂移 |

### E 接收（22 分项）✅（口径澄清）

| 指标 | 实测 | 说明 |
|---|---|---|
| E-01 图像接收丢帧 | 3.2~4.4%（best_effort 订阅端） | **丢在订阅端**（发布完整），判据对轻量订阅者结构性不可达 |
| E-02 IMU 接收 | **0** | ✅ |
| E-03 单调/重复 | 全 topic **0/0** | ✅ |
| E-04 接收延迟 | DDS 段 1.2~2.1ms（与消息体积线性）；t0→recv 14.1ms | ✅ 分段自洽 |
| E-05 带宽 | 满配发布 ≈375MB/s；IMU 工具实测与理论精确吻合 | ✅ |
| E-06 多订阅 1→4 | fps 不变、延迟 +2.1ms（≤10ms） | ✅ |
| E-10 info 配套 | 交集内 stamp **100% 相等** | ✅ |

### 门槛与长稳 ✅

| 项 | 实测 |
|---|---|
| 资源 30min | RSS/fd 零增长、CPU p50 100%（忙循环）、57~78°C |
| 启停 8 次 | **8/8**，零进程/设备残留 |
| **8h 长稳** | RESULT=OK：30.000fps 恒定、零取帧失败、零告警、RSS +2.13%（缓涨观察项）、IMU 丢样 0.27% 与短测一致；queue_recycle 9.4% 属缓冲周转非丢帧（前 2h 峰值后收敛） |

## 3. 问题清单（按优先级）

| # | 级别 | 问题 | 证据 | 建议 |
|---|---|---|---|---|
| P1 | **高** | `frame_ts_type=realtime` 的 stamp 被 int32 溢出回退 **−2.147s**（`exposure_time` 饱和 INT32_MAX） | 10373/10373 帧差值恒为 2147483647ns（05 轮） | 修 `hobot_mipi_cap_iml.cpp:650`：int64 计算并判 trig 有效性 |
| P2 | 高 | `trig_valid` 假阳性：LPWM 关闭后 `trig_tv` 残留旧值，判据（仅非零）恒真 | 05 轮 C04/C08；陈旧值=上次 LPWM 末拍+33.33ms | 驱动侧关闭时清零，或判据加单调推进 |
| P3 | 中 | **bgr8+combine 1088×1280@30 不可用**：拼接取帧 106ms、17% 拼接帧未发布（recycled 888） | 08 轮 C04B | 文档标注限制；优化 BGR 转换或降帧率 |
| P4 | 中 | IMU 需 **root**（/dev/iio* root-only），非 root 静默无数据 | 历轮 imu_topic_empty → root 后消失 | udev 规则或文档说明 |
| P5 | 中 | 双目相位每次启动随机两档：+3.0ms（对齐）/−13.7ms（半帧错档），约各半 | 07 轮 5 run；LPWM 可消除 | 需严格同步的部署用 lpwm；或定位 pipe 启动时序 |
| P6 | 中 | B-02 的 1ms 口径在 stamp 级不达（2.96ms）；触发级为 0、lpwm stamp 级 1.28ms=AE 曝光差 | 06/07 轮 | 口径定为触发级或开 `sync_ae` 复测 |
| P7 | 中 | IMU 丢样 0.27% > 0.01% 期望（内核侧 39.5ms 批量缺口、无读失败） | 10/11/8h 轮一致 | 驱动/内核侧排查（IIO 缓冲水位） |
| P8 | 低 | 辅流实测 14.0fps vs 标称 15（间隔双峰 67/83ms，VSE 分频周期性少帧） | 07/8h 轮 | 文档标注或调 VSE 分频 |
| P9 | 低 | 首帧 5.03s > 2s 期望（GDC bin 生成 2.58s 为一次性初始化） | 01 轮 | 预生成 GDC bin 缓存可降至 ~2.4s |
| P10 | 低 | RSS 8h +2.13% 缓涨（外推 24h≈6.4% 或触线） | 8h 轮 | 换版本时复测 24h |
| P11 | 低 | recorder `info_topics: []` 空数组启动崩溃；`ros2 topic bw` 数值仅作下限（观测者丢样） | 17 轮/8h 轮 | 工具注记 |

## 4. 遗留（本轮范围外）

- 21-combine-content（拼接内容相关性，需抓图存盘）
- 24h/多板/温漂；S100/S600 平台的 exposure 埋点移植
- P1/P2 的代码修复（评测只记录证据）
- E-01 的 reliable QoS 订阅者接收质量（需防反压设计）

## 5. 评测资产使用指南（换 sensor / 改配置）

```bash
# 1) 写一个用例 YAML（params 全量给 launch，thresholds 按需覆盖默认值）
#    模板：eval/cases/C04.yaml（含阈值覆盖示例）
# 2) 一键评测（板端；IMU 用例如需数据以 root 跑：ssh s600）
./eval/run/run_suite.py --suite smoke      # C04 60s 链路自检
./eval/run/run_suite.py --suite core       # 采集/同步核心基线 ~12min
./eval/run/run_suite.py --suite latency    # 延迟专题（含 bgr8/辅流）
./eval/run/run_suite.py C04 C07 --duration 60   # 任意组合
# 3) 汇总报告：eval/logs/suite_<名>_<时间戳>/suite_report.md（PASS/WARN/FAIL 三态）
# 4) 专项工具：run_bw.sh（带宽）、run_res.sh（长测资源）、run_startstop.sh（启停）、
#             run_stab8h.sh（8h 长稳，数据落 /tmp）
# 5) 新 sensor 首轮建议加跑 05（曝光/触发行为重标定：t0 偏移逐 boot 变化）
```

- 事件契约与阈值默认值：`include/hobot_mipi_eval_log.hpp`、`eval/tools/analyze/thresholds.py`
- 全部分析可离线重算：`analyze.py --csv <recorder.csv> --log <mipi_cam.eval.log>`（30min 数据 ~13s）

## 6. 轮次索引

| 轮 | 报告 | 轮 | 报告 |
|---|---|---|---|
| foundation | `reports/foundation/` | 10 IMU 时序 | `reports/10-imu-timing/` |
| 01 启动 | `reports/01-startup/` | 11 IMU 有效性 | `reports/11-imu-validity/` |
| 02 帧时序 | `reports/02-frame-timing/` | 13 Cam-IMU | `reports/13-cam-imu-basic/` |
| 03 丢帧 | `reports/03-drop/` | 14 漂移(30min) | `reports/14-cam-imu-drift/` |
| 04 分辨率 | `reports/04-resolution/` | 15 接收对账 | `reports/15-recv-drop/` |
| 05 曝光 | `reports/05-exposure/` | 16 接收延迟 | `reports/16-recv-latency/` |
| 06 双目同步 | `reports/06-stereo-sync/` | 17+18 带宽/负载 | `reports/17-18-bandwidth-multisub/` |
| 07 拼接 | `reports/07-combine/` | 19+20 资源/启停 | `reports/19-20-resources-startstop/` |
| 08 设备延迟 | `reports/08-device-latency/` | 8h 长稳 | `reports/longrun-8h/` |

（12 并入 10/11；09 并入 05/08/13）
