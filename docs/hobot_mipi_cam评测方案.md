# hobot_mipi_cam 综合评测方案

| 项 | 内容 |
|---|---|
| 被测对象 | `mipi_cam`（源码 `src/box/hobot_sensor/hobot_mipi_cam`，S100 后端） |
| 目标平台 | S100（board profile `s100-ngy-humble-3`，tros humble） |
| 传感器 | MIPI/GMSL camera（双目 rig，含 EEPROM 标定）+ icm42688 IMU |
| 文档版本 | v1.0（2026-11-09），验收阈值为初始线，首轮基线测试后修订 |
| 关联文档 | `cc_ws_s100_3/IMU与Camera评测方案_正式版.md`（Camera-IMU 时间同步方法论，本文引用不重复） |

## 1. 评测目标与范围

### 1.1 目标

1. **设备自身**：驱动在板端的数据通路稳定性（出帧、丢帧、队列）、资源占用、启停与长稳。
2. **ROS2 接收端**：外部订阅者看到的频率、延迟、消息正确性。
3. **码流矩阵**：单目主/辅码流、双目左右目、左右目拼接（combine）、各辅码流（sub_*）。
4. **IMU**：`/imu_data` 频率、数据质量、外参发布。
5. **时间同步**：图像-图像（左右目/主辅/拼接）、图像-IMU 的时间戳对齐精度，含时间戳域一致性审计。

### 1.2 数据通路（指标分解的依据）

```
sensor ─> VIN(max(fps,sub_fps)) ─> ISP/PYM/GDC ─> multiFrameTask(memcpy入队)
                                                  ├─ v_buff_que_manger_[i]      (主码流, 深度≈3)
                                                  ├─ v_sub_buff_que_manger_[i]  (辅码流)
                                                  └─ v_frame_que_[i] ─> 拼接任务 ─> combine_que
发布线程(每topic一个): getFrame(队列取帧) ─> [bgr8:NEON转换] ─> publish ─> DDS/hbmem ─> 订阅端
IMU: IIO/event0 ─> read_imu_data线程 ─> /imu_data + /imu_extrinsic + static TF
```

关键实现事实（评测设计的依据）：

- 底层 VIN 以 `max(fps, sub_fps)` 统一出帧，主/辅码流为**同源同时间戳**（同一 `out_img.info.timestamps`）。
- 发布端按"发布时刻表"抽帧：主码流 topic 按 `framerate`、辅码流 topic 按 `sub_framerate` 发布。
- 内部队列有效深度 ≈3（`get_empty_buff` 在 `q_buff_.size()>=3` 时回收最旧帧），积压会引入最多 ~3 帧延迟。
- 图像消息时间戳：`frame_ts_type=sensor` 用内核帧时间戳；`realtime` 转系统实时钟。
- IMU 时间戳：IIO 采样时间戳 / 读取时刻 CLOCK_REALTIME（**与图像 sensor 模式时间戳域可能不同，见 8.1 审计项**）。
- hbmem 消息含自增 `index` 字段（订阅端可用作传输丢帧检测）；ros 模式 Image 无序号，靠时间戳连续性判丢帧。

### 1.3 Topic 清单（被测数据面）

| 码流 | io_method=ros | io_method=shared_mem | camera_info |
|---|---|---|---|
| 单目主 | `image_raw` | `hbmem_img` | `image_raw/camera_info` |
| 单目辅 | `sub_image_raw` | `sub_hbmem_img` | `sub_image_raw/camera_info` |
| 双目左 | `image_left_raw` | `hbmem_left_img` | `image_left_raw/camera_info` |
| 双目右 | `image_right_raw` | `hbmem_right_img` | `image_right_raw/camera_info` |
| 拼接 | `image_combine_raw` | `hbmem_combine_img` | `image_combine_raw/left|right/camera_info` |
| 各辅码流 | 对应 `sub_*` 前缀 | 对应 `sub_hbmem_*` | 对应 `sub_*/camera_info` |
| IMU | `/imu_data`（sensor_msgs/Imu） | — | `/imu_extrinsic`（TransformStamped，与 IMU 同频）+ static TF `camera_link→imu_link` |

> `dual_combine=0/1/2` 决定双目下实际存在的 topic 集合（0=仅左右，1=左右+拼接，2=仅拼接）。

## 2. 评测环境与配置矩阵

### 2.1 环境要求

- 板端：S100 开发板，`s100-ngy-humble-3` profile，NFS 共享目录跑程序；测试期间固定 CPU 调频 governor（performance），关闭无关进程。
- 主机侧：工作区 `cc_ws_s100_3/tros_ws`，交叉编译产物 `install/lib/libmipi_cam_plugin.so`。
- 时钟：**延迟/同步测试必须在板端本机订阅**（避免跨机时钟误差）；跨机测试仅用于吞吐观察。
- 环境变量：shared_mem 模式须 `RMW_FASTRTPS_USE_QOS_FROM_XML=1`。

### 2.2 配置矩阵

| 维度 | 取值 |
|---|---|
| device_mode × dual_combine | single；dual×0；dual×1（左右+拼接）；dual×2（仅拼接） |
| sub_stream_enable | true / false（true 时全码流 6 个图像 topic） |
| 帧率组合 (framerate/sub_framerate) | 30/30（等频）、30/15（整数分频）、**7/8（非整数近邻，帧率控制精度专项）** |
| out_format | nv12（性能基线）、bgr8（转换开销压力场景） |
| io_method | ros、shared_mem |
| frame_ts_type | sensor（默认）、realtime（时间戳域审计） |
| 负载 | 空载；CPU 压力（stress-ng 占 50%/70% 核）；满配码流 |

典型全配场景（性能上限基线）：`dual + dual_combine=1 + sub_stream_enable=true + 1920x1080 + nv12 + 30/15fps + ros + IMU 200Hz`。

## 3. 设备自身（板端）评测

### 3.1 出帧与帧率（F 组指标）

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| F-01 VIN 实际出帧率 | `getImage` 日志的帧 ts 序列 / debugfs sensor 帧计数 | = max(fps, sub_fps)，偏差 ≤2% |
| F-02 主码流发布频率 | 订阅端 30s 窗口计数（见 4.1） | \|实测−配置\|/配置 ≤ 2% |
| F-03 辅码流发布频率 | 同上 | ≤ 2% |
| F-04 非整数近邻帧率（7/8） | 订阅端 60s 窗口 | 主 7±0.2fps、辅 8±0.2fps（验证发布时刻表抽帧） |
| F-05 帧率无突发 | 发布间隔无 <0.5×标称周期的连发 | 无连续 2 帧间隔 <0.8×周期 |

### 3.2 丢帧与队列健康（D 组）

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| D-01 驱动丢帧率 | 订阅端按帧 ts 反推帧序：`Δts/标称周期` 四舍五入求缺失帧数 | ≤0.1%（24h 长稳 ≤0.5%） |
| D-02 hbmem 传输丢帧 | 订阅端校验 `index` 连续性 | 记录基线；SensorDataQoS 下允许但需量化 |
| D-03 getFrame 失败计数 | 日志 `grab failed` / `select timeout` 计数 | 24h 为 0 |
| D-04 队列积压 | 日志 `laps ms`（帧 ts→拷贝完成）分布 | p95 ≤ 1.5 帧周期（>3 帧周期说明积压） |

### 3.3 资源占用（R 组）

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| R-01 CPU（进程级） | `pidstat -p <pid> 1`，区分空载/压力/满配场景 | 满配 nv12 ≤ 200%（6 核），bgr8 记录基线；版本回归偏差 ≤10% |
| R-02 CPU（线程级） | `top -H -p <pid>`：multiFrameTask / 发布线程×N / 拼接线程 / IMU 线程 | 各发布线程均衡，无单线程 >60% |
| R-03 内存 RSS | `pidstat -r`，24h 采样 | 稳态波动 ≤5%，无单调增长（泄漏） |
| R-04 fd/句柄 | `ls /proc/<pid>/fd \| wc -l`，启停 50 次前后 | 无增长 |
| R-05 温度与降频 | thermal zone + `cpufreq` 曲线，满配 2h | 高温限频场景下 F-02/03 仍达标 |

### 3.4 启停与长稳（S 组）

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| R-06 首帧延迟 | launch 起始 → 订阅端首条图像消息 | ≤ 5s |
| R-07 启停稳定性 | 连续 50 次 start/stop（launch 起停） | 成功率 100%，无进程残留、无 sensor 总线挂死 |
| R-08 长稳 | 满配 24h（抽 72h）：D-01/R-03/D-03 复测 | 全部达标，无重启 |

## 4. ROS2 接收端评测

### 4.1 频率与间隔稳定性（订阅端视角）

测量：板端 recorder（见 11.1）订阅全部 topic，记录 `(topic, header.stamp, recv_monotonic_ns)`；辅以 `ros2 topic hz` 粗测交叉验证。

| 指标 | 定义 | 初始验收线 |
|---|---|---|
| S-01 平均频率 | 60s 计数/时长 | 见 F-02/03/04 |
| S-02 间隔分位数 | 相邻消息 recv 间隔 p50/p95/p99/max | p99 ≤ 1.5×标称周期；max ≤ 3× |
| S-03 频率稳定性 | 逐秒频率的 std | ≤ 5% 标称值 |

### 4.2 端到端延迟（L 组）

定义：`延迟 = recv_clock_ns − header.stamp`（**同机同钟域前提成立时**；钟域审计见 8.1）。

分解观测点：
- 曝光/帧完成 → VIN 时间戳（sensor ts）：由帧周期界定；
- sensor ts → 拷贝完成：驱动日志 `laps ms`；
- 拷贝完成 → 订阅端收到：recorder 差值。

| 指标 | 场景 | 初始验收线（同机） |
|---|---|---|
| L-01 端到端 p50/p95 | nv12 + ros + 1080p30 单目 | p50 ≤ 70ms，p95 ≤ 120ms |
| L-02 端到端 p50/p95 | bgr8 同上 | p95 ≤ 150ms（含 NEON 转换） |
| L-03 全配（双目+拼接+辅流） | nv12 + ros | p95 ≤ 180ms |
| L-04 shared_mem 对比 | hbmem vs ros 同配置 | hbmem p50 ≤ ros p50（记录差值） |
| L-05 camera_info 配套延迟 | info 与 image 收到时刻差 | ≤ 2ms，stamp 与 image 100% 相等 |

> 阈值依据：队列深度 3 帧上限 + 1 帧处理周期 + DDS 序列化/拷贝；首轮实测后修订为紧线。

### 4.3 消息正确性（Q 组）

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| Q-01 元数据一致性 | width/height/encoding/step 与 launch 参数逐帧比对 | 100% 一致 |
| Q-02 图像内容 | 抽帧（每分钟 1 帧）人工/自动检查：撕裂、错位、花屏、左右目错换 | 0 缺陷 |
| Q-03 NV12→BGR8 转换 | 已知色卡/渐变图对比标准转换 | 最大通道误差 ≤2 |
| Q-04 内参正确性 | `camera_info` 的 K/D/P/R vs 标定文件/EEPROM | 相对误差 ≤0.1% |
| Q-05 辅码流内参缩放 | `sub_*` 内参 vs 主内参×缩放比 | 同上（驱动 `scaleSubStreamCamInfo`） |
| Q-06 frame_id | 全 topic `header.frame_id` = 配置值 | 100% |

### 4.4 扩展性（X 组）

| 指标 | 场景 | 初始验收线 |
|---|---|---|
| X-01 多订阅者 | 同一 topic 1/2/4 个订阅者 | 频率不降，延迟增量 ≤10ms |
| X-02 rosbag 录制中 | 录制 + 实时订阅并存 | 频率不降（允许延迟上浮 ≤30ms） |

## 5. 双目专项（左右目）

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| ST-01 左右同源配对率 | 按 ts 精确相等配对（同源同 ts） | 配对率 = 100%（≥99.9%，缺失按 D-01 计丢帧） |
| ST-02 左右 ts 差 | 配对帧 ts 差分布 | 期望 0；上限 1ms |
| ST-03 左右频率一致 | 两 topic 独立测频 | 差 ≤ 0.5% |
| ST-04 图像内容同步 | 静态棋盘格 + 摆动棋盘格（亚像素角点互相关）；`sync_ae/awb/ccm` 开/关对比 | 摆动场景左右角点位置差换算时间 ≤ 1/2 帧周期 |
| ST-05 亮度一致性 | 同帧左右灰度均值/直方图对比（sync_awb/ccm 开启） | 均值差 ≤5%（主观无可感色差） |
| ST-06 双目标定质量 | camera_info 外参 → 立体校正后重投影误差（棋盘格） | ≤ 0.5px |

## 6. 拼接专项（combine）

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| CB-01 拼接频率 | 订阅端测频 | = 左右频率（跟随 VIN），偏差 ≤2% |
| CB-02 拼接额外延迟 | 同帧 `combine` 与 `left` 收到时刻差 | ≤ 10ms |
| CB-03 拼接完整性 | 抽样：combine 左半区/右半区 与 同帧 left/right 对应区域相关性 | 相关系数 ≥0.98，无错位/重叠/缝隙 |
| CB-04 拼接 ts 一致性 | combine ts vs 同帧 left ts | 相等（同源复制） |
| CB-05 双 camera_info | `image_combine_raw/left|right/camera_info` 内容 vs 左右目各自内参 | 一致（Q-04 标准） |
| CB-06 dual_combine=2 | 仅拼接模式下频率/延迟/内容复测 | 同 CB-01/02/03 |

## 7. IMU 专项

参数事实：icm42688，ODR 200Hz（可选 500Hz），量程 ±16g / ±2000dps（scale：accel 0.004788403 m/s²/LSB，gyro 0.001065264 rad/s/LSB）。

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| I-01 IMU 频率 | `/imu_data` 60s 计数 | 200Hz ±0.5% |
| I-02 IMU 间隔 | 间隔 p95/p99/max | p99 ≤ 7.5ms；max ≤ 15ms |
| I-03 时间戳单调性 | 逐条校验 ts 递增 | 100% 单调（回退即 FAIL） |
| I-04 静态噪声 | 静置 30min：gyro/accel std + Allan 方差 | std ≤ datasheet 噪声密度×√BW×3（gyro ≈0.0028 dps/√Hz、accel ≈70 µg/√Hz 量级，按手册复核） |
| I-05 静态偏置 | 均值 vs datasheet 零偏 | accel ≈9.8 m/s²（模长），gyro ≤0.01 rad/s；温漂记录 |
| I-06 数据合理性 | 静置模长、六面翻转重力方向 | ±5% |
| I-07 外参发布 | `/imu_extrinsic`、TF vs EEPROM 标定值（含 RDF→FLU 转换核对） | 数值一致 |
| I-08 外参频率 | TransformStamped 计数 | 与 IMU 同频（同线程发布） |

## 8. 时间同步专项

### 8.1 时间戳域一致性审计（P0，先于一切同步测试）

各流时间戳来源不同（图像 sensor 模式 = 内核帧时间戳；realtime 模式 = 系统实时钟；IMU = IIO 采样时间戳或读取时刻 CLOCK_REALTIME），**域不一致会表现为常量级偏差**。

审计方法：对每个 topic 记录 `|header.stamp − recv 端 now()|` 的中位数，与"系统 uptime vs epoch"对照：
- 中位数 ≈ uptime → monotonic 域；
- 中位数 ≈ 0 → realtime 域。

| 指标 | 初始验收线 |
|---|---|
| T-01 同域性 | 同一次评测中，图像 topic 与 IMU 的 `header.stamp` **全部同域**；跨域组合（如 sensor 图像 + REALTIME IMU）必须给出换算关系并单独记录为缺陷/已知限制 |
| T-02 实时性 | 同域前提下，`now() − stamp` 中位数 ≤ 2×标称周期（无陈旧帧） |

### 8.2 图像-图像同步

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| T-03 主-辅码流同帧 | `image_raw` vs `sub_image_raw` 按 ts 配对差 | 相等（同源）；上限 1ms |
| T-04 左右目 | 见 ST-02 | ≤ 1ms |
| T-05 拼接-单目 | 见 CB-04 | 相等 |

### 8.3 图像-IMU 同步（引用正式版方案）

方法论（采样对齐、曝光中点校正、L1/L2/L3 分级）沿用 `IMU与Camera评测方案_正式版.md` 第 4 章，本方案负责落地量化：

| 指标 | 测量方法 | 初始验收线 |
|---|---|---|
| T-06 最近邻 IMU 间隔 | 每帧图像 ts 与最近 IMU ts 差分布 | p99 ≤ 1/2 IMU 周期（2.5ms @200Hz） |
| T-07 物理激励对齐 | LED 闪光（图像亮度突变）+ 同步敲击（IMU 冲击）事件时刻差；≥30 组取 p95 | ≤ 5ms（软件时间戳路径）；目标 ≤2ms（含曝光中点校正后） |
| T-08 长时漂移 | 1h 连续记录 nearest-neighbor 差的趋势斜率 | \|漂移\| ≤ 0.1ms/min（同钟域应无漂移；有斜率即钟域问题） |

## 9. 量化指标汇总（验收总表）

> P0=阻塞发布，P1=需修复，P2=记录基线。阈值 v1.0 为初始线，首轮基线后修订。

| 组 | 编号 | 指标 | 初始验收线 | 级别 |
|---|---|---|---|---|
| 帧率 | F-02/03 | 主/辅发布频率精度 | ≤2% | P0 |
| 帧率 | F-04 | 7/8 非整数近邻 | 7±0.2 / 8±0.2 fps | P0 |
| 丢帧 | D-01 | 驱动丢帧率 | ≤0.1%（24h ≤0.5%） | P0 |
| 丢帧 | D-03 | getFrame 失败/超时 | 24h = 0 | P0 |
| 延迟 | L-01~03 | 端到端 p95 | 120~180ms（分场景） | P0 |
| 延迟 | L-05 | info-image 配套 | stamp 100% 相等 | P1 |
| 正确性 | Q-01/02 | 元数据/内容 | 100% / 0 缺陷 | P0 |
| 正确性 | Q-04/05 | 内参/缩放 | ≤0.1% | P1 |
| 双目 | ST-01/02 | 配对率/ts 差 | ≥99.9% / ≤1ms | P0 |
| 双目 | ST-04 | 内容同步 | ≤1/2 帧周期 | P1 |
| 拼接 | CB-02/03 | 额外延迟/区域一致 | ≤10ms / r≥0.98 | P1 |
| IMU | I-01/02 | 频率/间隔 | ±0.5% / p99≤7.5ms | P0 |
| IMU | I-03 | 单调性 | 100% | P0 |
| 同步 | T-01 | 时间戳域一致 | 全部同域 | P0 |
| 同步 | T-06 | 图像-IMU 最近邻 | p99 ≤2.5ms | P0 |
| 同步 | T-07 | 物理激励对齐 | p95 ≤5ms | P0 |
| 资源 | R-01/03 | CPU/RSS | ≤200%（满配）/ 24h ≤5% | P1/P0 |
| 稳定 | R-07/08 | 启停/长稳 | 100% / 全项达标 | P0 |

## 10. 测试用例清单

| 用例 | 场景（配置） | 覆盖指标 | 时长 |
|---|---|---|---|
| TC-01 | 单目 1080p nv12 30fps ros + IMU | F-02, S-01/02, L-01, Q-01/02, I-01/02, T-01/06 | 10min |
| TC-02 | TC-01 + out_format=bgr8 | L-02, Q-03, R-01 | 10min |
| TC-03 | 单目 + sub_stream（30/15） | F-03, T-03, Q-05 | 10min |
| TC-04 | 单目 帧率 7/8 | F-04, F-05 | 60min |
| TC-05 | 双目 dual_combine=1（nv12 ros） | ST-01~05, L-03, R-01/02 | 15min |
| TC-06 | TC-05 + sub_stream 全配 | 全量基线：F/D/L/R + ST + CB | 30min |
| TC-07 | TC-05 + io_method=shared_mem | L-04, D-02, X-01 | 15min |
| TC-08 | dual_combine=2 仅拼接 | CB-06 | 10min |
| TC-09 | TC-06 + CPU 压力 70% | S-02, L-03, R-05 | 15min |
| TC-10 | frame_ts_type=realtime 全配 | T-01/02（域审计） | 10min |
| TC-11 | 物理激励：LED 闪光 + 敲击 ≥30 组 | T-07 | 30min |
| TC-12 | 长稳：TC-06 配置连续运行 | D-01/03, R-03/05/08 | 24h |
| TC-13 | 启停 50 次 | R-06/07, R-04 | ~2h |
| TC-14 | 多订阅者 1/2/4 + rosbag 并录 | X-01/02 | 15min |
| TC-15 | 静置 IMU 30min + 六面翻转 | I-04~06 | ~1h |
| TC-16 | 标定验证：棋盘格采图 | Q-04, ST-06, CB-03/05 | 人工 |

执行顺序：TC-10（域审计）→ TC-01 → TC-02~09 → TC-11 → TC-15/16 → TC-14 → TC-13 → TC-12。

## 11. 工具与自动化

### 11.1 板端 recorder（核心采集工具，建议 C++）

订阅全部目标 topic（QoS 与被测端匹配），每条消息落一行 CSV：

```
topic, seq_or_index, header_stamp_ns, recv_monotonic_ns, width, height, encoding
```

要点：
- 用 `CLOCK_MONOTONIC` 记接收时刻，与分析统一域；
- 图像数据不落盘（只记录元数据），避免 recorder 自身成为负载；hbmem 通路校验 `index`；
- Python(rclpy) 仅够 IMU/单 topic；6×1080p30 图像必须 C++，否则 recorder 丢帧污染结果。

### 11.2 分析脚本（离线）

输入 recorder CSV，输出指标报告：
- 频率/间隔统计（均值、逐秒频率、p50/p95/p99/max）；
- 端到端延迟分位数 + 时间序列；
- 帧序反推丢帧清单（D-01）；
- 左右/主辅/拼接 ts 配对与差值分布（ST/CB/T 组）;
- 图像-IMU 最近邻差分布与漂移斜率（T-06/08）。

### 11.3 现成工具链

| 用途 | 工具 |
|---|---|
| 板端执行与日志收集 | `run-tros-s100` skill（结果落共享目录） |
| topic 时序图 | `topic-timing-plot` skill（间隔 + 延迟 PNG） |
| CPU 分析 | `generic-cpu-profiler` / `pidstat` / `top -H` |
| 粗测交叉验证 | `ros2 topic hz/delay`、`ros2 topic echo` |
| 原始记录 | rosbag2（仅元数据型小 topic 全录，图像 topic 按需） |

### 11.4 报告模板

每次执行输出：结论（PASS/FAIL 总表）→ 环境与配置指纹（git commit、launch 参数、板端温度/频率）→ 指标明细表 → 图表（间隔直方图、延迟时序、ts 差分布、CPU/RSS 曲线）→ 原始数据路径 → 异常事件清单。

## 12. 测量注意事项（坑）

1. **跨机时钟**：延迟/同步指标一律板端本机测；跨机订阅只看吞吐。
2. **丢帧归因**：SensorDataQoS 下订阅端少帧可能是传输丢弃而非驱动丢帧——用 hbmem `index` / 帧序反推区分，必要时换 Reliable QoS 复测。
3. **recorder 自干扰**：图像序列化拷贝大，recorder CPU 要监控（≤30% 单核），否则数据作废。
4. **bgr8 放大延迟**：bgr8 每帧 NEON 转换在发布线程内，延迟与 CPU 显著高于 nv12，结果须标注 out_format。
5. **负载公平**：测延迟时固定 governor、关闭日志刷屏（log_level=warn，避免每帧 INFO 日志）。
6. **vin 帧率 ≠ 配置**：sensor 模式不支持的目标帧率会落到最近支持档（如 8→7.5），先测 F-01 再解读 F-02/03。
7. **首帧瞬态**：启动后前 5s 数据剔除（ISP/AE 收敛、队列预热）。

## 13. 待确认问题

1. `frame_ts_type=sensor` 的内核时间戳与 IMU IIO 时间戳的钟域关系（T-01 审计后确定是否需要驱动侧统一到 REALTIME）。
2. 曝光时间是否可从 ISP 元数据获取（用于 T-07 曝光中点校正；当前消息未携带）。
3. 图像消息是否值得补充 `frame_id` 序号字段（让 D-01 免于 ts 反推；涉及消息兼容性）。
4. bgr8 满配场景 CPU 初始验收线（先跑基线再定）。
