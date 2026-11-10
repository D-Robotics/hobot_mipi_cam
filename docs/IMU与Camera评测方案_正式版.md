# Camera 与 IMU 评测方案

**—— ROS 2 多传感器数据采集与时间同步评测系统（Camera + IMU Benchmark）**

| 文档属性 | 内容 |
| --- | --- |
| 版本 | V1.0 |
| 日期 | 2026-11-04 |
| 状态 | 草案（待评审） |
| 适用范围 | hobot_mipi_cam Camera 链路、IMU 驱动链路及 Camera-IMU 时间同步评测 |

**修订记录**

| 版本 | 日期 | 修订内容 | 修订人 |
| --- | --- | --- | --- |
| V1.0 | 2026-11-04 | 由讨论稿整理成文 | — |

---

## 1. 背景与目标

在原有 ROS 2 Camera Benchmark 的基础上，将 IMU 评测和 Camera-IMU 时间戳同步评测纳入进来，升级为一套 **ROS 2 多传感器数据采集与时间同步评测系统（Camera + IMU Benchmark）**。

本方案需要特别注意：不仅要评测 IMU 和 Camera 各自的数据质量、频率和延迟，还要**独立评测两者之间的时间同步精度**。

系统包含三个核心模块：

1. **IMU 单体评测**：采样频率、数据精度、噪声、零偏、数据连续性、时间戳准确性；
2. **Camera-IMU 时间同步评测**：时间戳偏差、同步抖动、长期漂移、同步稳定性；
3. **ROS 2 数据链路评测**：消息发布时间、接收时间、DDS 传输延迟和消息丢失。

## 2. 整体评测架构

### 2.1 系统架构

```
Hardware Sensors
        |
   +----+----+
   |         |
 Camera     IMU
   |         |
Camera Driver  IMU Driver
   |         |
   v         v
/camera/image_raw   /imu/data_raw
   |         |
   +----+----+
        |
  ROS 2 DDS Layer
        |
   +----+----+
   |         |
Camera Subscriber  IMU Subscriber
   |         |
   +----+----+
        |
 Evaluation Node
        |
  +-----+-------+-----+
  |      |            |
Camera  IMU       Time Sync
Analysis Analysis  Analysis
  |      |            |
  +-----+-------+-----+
        |
 Report Generator
```

### 2.2 关键设计原则：硬件时间戳优先

同步评测应**尽量使用硬件时间戳和共同硬件时钟作为依据，而不是直接使用 ROS 2 消息到达时间**。

原因：DDS 传输延迟、操作系统调度和线程阻塞都会影响消息到达时间。仅比较 Subscriber 收到消息的时间，不能准确代表 Camera 和 IMU 的采集同步误差。

## 3. IMU 单体评测方案

### 3.1 IMU 功能测试

主要评测 ROS 2 IMU Driver 是否正确采集和发布数据。

标准 ROS 2 消息类型：`sensor_msgs/msg/Imu`

Topic 约定：

- `/imu/data_raw` —— 原始数据评测；
- `/imu/data` —— 经过滤波或融合处理的数据评测；
- `/imu/mag` —— 磁力计数据（如设备支持）。

| 测试项 | 测试内容 | 预期结果 |
| --- | --- | --- |
| 节点启动 | 启动 IMU Driver | 正常启动 |
| Topic 发布 | 检查 IMU Topic | 正常发布 |
| 消息格式 | 检查 Imu Message | 字段正确 |
| 线加速度 | Acceleration XYZ | 数值有效 |
| 角速度 | Angular Velocity XYZ | 数值有效 |
| Orientation | 四元数 | 符合设备能力 |
| Frame ID | 检查 frame_id | 与 TF 配置一致 |
| 时间戳 | 检查 Header Stamp | 单调且有效 |
| 采样频率 | 100/200/400/800 Hz | 满足配置 |
| 数据连续性 | 检查消息间隔 | 无异常中断 |

> **注意**：部分 IMU 不提供 Orientation，或只提供原始加速度和角速度，此时不应将 Orientation 字段作为强制通过条件。

### 3.2 IMU 采样频率测试

支持以下测试档位：

| IMU 配置频率 | 目标测试场景 |
| --- | --- |
| 100 Hz | 基础采样 |
| 200 Hz | 常规机器人应用 |
| 400 Hz | 高频运动估计 |
| 800 Hz | 高性能视觉惯性系统 |
| 1000 Hz | 高速运动估计 |

**评测指标**：

- Average Frequency（平均频率）
- Minimum Frequency / Maximum Frequency（最小/最大频率）
- Frequency Jitter（频率抖动）
- Message Drop Rate（丢包率）
- Maximum Message Interval（最大消息间隔）

**计算公式**：

瞬时采样频率：

$$f_i = \frac{1}{t_i - t_{i-1}}$$

平均采样频率：

$$f_{avg} = \frac{N-1}{t_N - t_1}$$

采样间隔抖动：

$$J_i = \Delta t_i - \frac{1}{f_{target}}$$

> 建议统计 **P50、P95、P99 和 Max** 分位数，而不只统计平均频率。

### 3.3 IMU 数据精度测试

精度测试分为静态测试与动态测试两类。

#### A. 静态测试

将 IMU 固定在稳定平台上，保持静止。

| 指标 | 测试内容 |
| --- | --- |
| Accelerometer Bias | 加速度计零偏 |
| Gyroscope Bias | 陀螺仪零偏 |
| Accelerometer Noise | 加速度计噪声 |
| Gyroscope Noise | 陀螺仪噪声 |
| Data Stability | 数据稳定性 |
| Temperature Drift | 温度变化引起的零偏变化 |

静止状态下，加速度计应主要感知重力，陀螺仪角速度应接近零。

#### B. 动态测试

使用转台或运动平台进行测试。

| 测试项 | 测试方法 |
| --- | --- |
| 角速度精度 | 与转台标准角速度对比 |
| 加速度精度 | 与标准加速度对比 |
| 轴向一致性 | X/Y/Z 轴分别测试 |
| 动态响应 | 阶跃运动测试 |
| 饱和检测 | 检查量程边界 |
| 运动连续性 | 连续运动数据采集 |

#### C. IMU 时间戳精度

重点检查：

- Timestamp 是否单调递增；
- 时间戳是否存在重复；
- 时间戳是否存在跳变；
- 时间戳与真实采样时间是否一致；
- 系统运行一段时间后是否发生时钟漂移。

建议保存每条 IMU 消息的以下字段：

```text
header.stamp
callback_receive_time
sequence_id
angular_velocity
linear_acceleration
```

如果 Driver 支持硬件时间戳，还应额外记录原始 Hardware Timestamp。

## 4. Camera-IMU 时间戳同步评测

本部分是整套评测方案中最重要的新增内容。

### 4.1 需要评测的同步误差

建议拆分为四个维度：

| 评测指标 | 含义 |
| --- | --- |
| Timestamp Offset | Camera 与 IMU 时间戳偏差 |
| Timestamp Jitter | 同步误差波动 |
| Clock Drift | 长时间运行时钟漂移 |
| Synchronization Stability | 同步稳定性 |

典型场景：Camera 输出频率 30 FPS，IMU 输出频率 400 Hz。

> **注意**：30 FPS Camera 与 400 Hz IMU 的采样时刻通常不会天然重合。因此**不能**简单地将每一帧 Camera 时间戳与最近的一条 IMU 时间戳相减并认定为同步误差。更合理的做法是建立统一时间基准，并根据硬件同步事件或已知采集时刻计算两者的时间偏差。

### 4.2 同步方案分级

| 等级 | 同步方式 | 评测能力 |
| --- | --- | --- |
| L1 | 软件时间戳 | 评估软件链路时间一致性 |
| L2 | 硬件时间戳 + 时钟映射 | 评估真实采集时间一致性 |
| L3 | PPS / Trigger / FSYNC | 评估硬件同步精度（**推荐**） |

对于机器人 VIO、SLAM 和自动驾驶应用，建议优先采用 L3 硬件同步方案。

#### L1：软件时间戳同步

Camera 和 IMU 分别通过 ROS 2 Driver 产生时间戳，直接比较：

$$\Delta t = t_{camera} - t_{imu}$$

- 优点：实现简单；不需要修改硬件；适合初步评测。
- 缺点：受系统调度影响；可能混入驱动处理延迟；无法准确区分真实采集时间和软件发布时间。

#### L2：硬件时间戳同步

Camera 和 IMU 使用硬件时间戳，分别映射到统一时间基准：

```text
Camera Hardware Clock ----+
                          |
                          +---- Common Time Base
                          |
IMU Hardware Clock -------+
```

需要测试：

- Hardware Timestamp 精度；
- Clock Mapping 误差；
- Clock Offset；
- Clock Drift；
- Clock Reset。

#### L3：硬件触发同步

推荐使用：

- Camera Trigger Input；
- IMU FSYNC；
- PPS；
- Shared Clock；
- PTP（支持硬件时间戳的设备）。

通过共同的硬件同步事件，建立 Camera 和 IMU 的时间关联。

> **注意**：PPS 主要提供周期性时间基准，并不自动意味着每一帧曝光和每一个 IMU 样本都在同一时刻；还需要明确 Camera 触发和 IMU 同步采样的具体机制。

### 4.3 同步误差计算

推荐定义 Camera 第 $i$ 帧的误差：

$$E_i = t_{camera,i} - t_{reference,i}$$

其中：

- $t_{camera,i}$：Camera 第 $i$ 帧的真实采集参考时刻；
- $t_{reference,i}$：统一时钟下对应的参考时刻。

对于 IMU 第 $j$ 个样本：

$$E_j = t_{imu,j} - t_{reference,j}$$

如果有共同的硬件同步事件，可以直接比较两端对同一个事件的时间记录。

Camera 与 IMU 的相对时间误差：

$$E_{sync} = E_{camera} - E_{imu}$$

统计量：

$$E_{mean} = \frac{1}{N}\sum_{i=1}^{N} E_{sync,i}$$

$$E_{std} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\left(E_{sync,i} - E_{mean}\right)^2}$$

另外还应统计：

- Mean Offset（平均偏差）
- Mean Absolute Error（平均绝对误差）
- RMS Error（均方根误差）
- Standard Deviation（标准差）
- P95 Error / P99 Error
- Maximum Absolute Error（最大绝对误差）

> 这里要区分两个概念：
> - **固定 Offset**：可以通过标定进行补偿；
> - **动态 Jitter**：反映同步稳定性，不能简单依靠固定 Offset 消除。

### 4.4 Camera 曝光时间的处理

这是非常容易被忽略的问题。

Camera 的 ROS 2 `header.stamp` 不一定代表曝光中点。不同 Driver 可能使用：

- 曝光开始时间；
- 曝光结束时间；
- 曝光中点；
- USB 数据接收时间；
- 驱动完成处理的时间。

因此建议在评测前统一定义：

$$\text{Camera Timestamp} = \text{曝光中点（Exposure Midpoint）}$$

对于**全局快门**：

$$t_{mid} = t_{start} + \frac{T_{exposure}}{2}$$

对于**滚动快门**，不同图像行的曝光时间不同，还需要加入行时间偏移：

$$t_{row} = t_{ref} + \Delta t_{row}$$

否则即使 ROS 2 时间戳完全一致，也可能存在真实的运动观测时间误差。

## 5. Camera + IMU 联合测试用例

| ID | 测试项目 | 测试内容 | 优先级 |
| --- | --- | --- | --- |
| IMU001 | IMU 启动测试 | Driver 启动及数据发布 | P0 |
| IMU002 | IMU 频率测试 | 100/200/400/800 Hz | P0 |
| IMU003 | IMU 数据完整性 | XYZ 数据及有效性 | P0 |
| IMU004 | IMU 时间戳 | 单调性、重复、跳变 | P0 |
| IMU005 | IMU 静态精度 | 零偏、噪声 | P1 |
| IMU006 | IMU 动态精度 | 转台测试 | P1 |
| IMU007 | IMU 长时间运行 | 24 小时稳定性 | P1 |
| SYNC001 | Camera-IMU Offset | 时间戳偏差 | P0 |
| SYNC002 | Camera-IMU Jitter | 同步抖动 | P0 |
| SYNC003 | Clock Drift | 时钟漂移 | P0 |
| SYNC004 | Timestamp Monotonic | 时间戳单调性 | P0 |
| SYNC005 | Hardware Trigger | 硬件触发同步 | P1 |
| SYNC006 | Long-term Sync | 长时间同步稳定性 | P1 |
| SYNC007 | CPU Load Sync | 高 CPU 负载下同步误差 | P1 |
| SYNC008 | Reconnection Sync | 设备重连后同步恢复 | P1 |

## 6. 自动化测试实现方案

建议在原有 Python + ROS 2 测试框架中增加一个 `sync_evaluator` 节点。

### 6.1 ROS 2 节点架构

```
camera_driver
     |
     v
/camera/image_raw
     |
     v
camera_monitor --------+
                       |
                       v
                 sync_evaluator
                       ^
                       |
imu_monitor -----------+
     ^
     |
/imu/data_raw
     |
imu_driver
```

| Node | 功能 |
| --- | --- |
| camera_monitor | 记录 Camera 时间戳及图像帧信息 |
| imu_monitor | 记录 IMU 时间戳及采样数据 |
| sync_evaluator | 计算同步误差 |
| performance_monitor | CPU、内存、网络监控 |
| report_generator | 生成评测报告 |

### 6.2 建议记录的数据结构

Camera CSV：

```csv
frame_id,header_stamp,hardware_stamp,receive_time,exposure_time
1,1000.033,1000.032,1000.040,0.010
2,1000.066,1000.065,1000.073,0.010
3,1000.100,1000.099,1000.108,0.010
```

IMU CSV：

```csv
seq,header_stamp,hardware_stamp,receive_time,gx,gy,gz,ax,ay,az
1,1000.001,1000.001,1000.003,0.01,0.02,0.01,0.1,0.2,9.8
2,1000.003,1000.003,1000.005,0.01,0.02,0.01,0.1,0.2,9.8
3,1000.006,1000.006,1000.008,0.01,0.02,0.01,0.1,0.2,9.8
```

以上数据仅为格式示例。

建议保留四种时间戳，以便进一步区分误差来源：

1. **Hardware Timestamp** —— 硬件采集时间；
2. **Header Timestamp** —— ROS 2 消息时间戳；
3. **Publish Timestamp** —— 消息发布时间;
4. **Receive Timestamp** —— Subscriber 接收时间。

由此可以区分：

- 传感器采集误差；
- Driver 时间戳误差；
- ROS 2 传输延迟；
- Subscriber 调度延迟。

### 6.3 同步误差计算示例（Python）

以下示例用于计算两组已经映射到同一时钟基准的时间戳偏差。实际评测时，应使用共同硬件事件或经过校准的时间映射关系。

```python
import numpy as np

def evaluate_sync(camera_times, imu_reference_times):
    """
    camera_times:
        Camera reference timestamps

    imu_reference_times:
        对应同步事件的 IMU reference timestamps

    两组数组必须对应相同的硬件同步事件，
    且已经映射到相同时间基准。
    """

    camera_times = np.asarray(camera_times)
    imu_times = np.asarray(imu_reference_times)

    if len(camera_times) != len(imu_times):
        raise ValueError("Timestamp count mismatch")

    if len(camera_times) < 2:
        raise ValueError("Insufficient samples")

    errors_ms = (camera_times - imu_times) * 1000.0

    result = {
        "mean_offset_ms": float(np.mean(errors_ms)),
        "mean_abs_error_ms": float(np.mean(np.abs(errors_ms))),
        "std_ms": float(np.std(errors_ms)),
        "rms_ms": float(np.sqrt(np.mean(errors_ms ** 2))),
        "p95_ms": float(np.percentile(np.abs(errors_ms), 95)),
        "p99_ms": float(np.percentile(np.abs(errors_ms), 99)),
        "max_abs_error_ms": float(np.max(np.abs(errors_ms))),
    }

    return result
```

> **特别注意**：不能直接把 30 FPS 的 Camera 时间戳数组和 400 Hz 的 IMU 时间戳数组按数组下标相减。两种传感器采样频率不同，需要先通过共同事件、硬件时钟或明确的时间映射关系建立对应关系。

## 7. 同步性能验收标准

建议建立三个级别的同步验收标准。以下为针对机器人视觉惯性系统的**初始工程目标**，并非 ROS 2 官方规范。

| 指标 | 基础级 | 良好级 | 优秀级 |
| --- | --- | --- | --- |
| 平均同步偏差 | ≤ 5 ms | ≤ 1 ms | ≤ 0.1 ms |
| 同步抖动 P99 | ≤ 5 ms | ≤ 1 ms | ≤ 0.2 ms |
| 最大同步误差 | ≤ 10 ms | ≤ 2 ms | ≤ 0.5 ms |
| 长期时钟漂移 | ≤ 1 ms/h | ≤ 0.1 ms/h | ≤ 0.02 ms/h |

> 上述指标应以硬件采集时间为基准，且测量设备自身的误差必须显著小于验收阈值。
> 如果使用软件时间戳，建议单独报告软件时间一致性，**不要**将其作为硬件同步精度的验收结论。

## 8. 同步可视化 Dashboard

评测平台建议提供四张核心图表（以下均为展示用途的**模拟数据**，不代表实际硬件测试结果）。

**图表 1：Camera 与 IMU 时间戳偏差趋势**（同步误差随时间变化）

| 时间 | Sync Error |
| --- | --- |
| 0 s | 0.1 ms |
| 10 s | 0.2 ms |
| 20 s | 0.1 ms |
| 30 s | 0.2 ms |
| 40 s | 0.1 ms |
| 50 s | 0.2 ms |
| 60 s | 0.1 ms |

**图表 2：同步误差分布**（不同误差区间的样本数量）

| 误差区间 | Sample Count |
| --- | --- |
| 0–0.1 ms | 850 |
| 0.1–0.2 ms | 120 |
| 0.2–0.5 ms | 25 |
| 0.5–1 ms | 4 |
| >1 ms | 1 |

**图表 3：Camera FPS 与 IMU Frequency**（两路传感器采样频率稳定性）

| 时间 | Camera | IMU |
| --- | --- | --- |
| 0 s | 30 FPS | 400 Hz |
| 10 s | 30 FPS | 400.1 Hz |
| 20 s | 30 FPS | 399.9 Hz |
| 30 s | 30 FPS | 400 Hz |
| 40 s | 30 FPS | 400.1 Hz |

**图表 4：Clock Drift**（时钟相对偏差随运行时间变化）

| 时间 | Clock Offset |
| --- | --- |
| 0 h | 0.1 ms |
| 1 h | 0.1 ms |
| 2 h | 0.2 ms |
| 3 h | 0.2 ms |
| 4 h | 0.2 ms |
| 5 h | 0.3 ms |

## 9. Benchmark 项目目录结构

建议最终项目组织如下：

```text
ros2_sensor_benchmark/
|
+-- config/
|   +-- camera_config.yaml
|   +-- imu_config.yaml
|   +-- sync_config.yaml
|
+-- launch/
|   +-- camera_test.launch.py
|   +-- imu_test.launch.py
|   +-- camera_imu_test.launch.py
|
+-- src/
|   +-- camera_monitor.py
|   +-- imu_monitor.py
|   +-- sync_evaluator.py
|   +-- performance_monitor.py
|   +-- timestamp_analyzer.py
|
+-- tests/
|   +-- test_camera.py
|   +-- test_imu.py
|   +-- test_sync.py
|   +-- test_stability.py
|
+-- tools/
|   +-- timestamp_analysis.py
|   +-- imu_noise_analysis.py
|   +-- generate_report.py
|
+-- results/
    +-- camera/
    +-- imu/
    +-- sync/
    +-- reports/
```

## 10. 评测层次划分

整个 Benchmark 分为四个层次：

| Level | 名称 | 内容 |
| --- | --- | --- |
| Level 1 | Sensor Function Benchmark | Camera 和 IMU 各自是否正常工作，数据是否正确、完整 |
| Level 2 | Sensor Performance Benchmark | FPS、IMU Frequency、Latency、Bandwidth、CPU、Memory |
| Level 3 | Camera-IMU Synchronization Benchmark | Timestamp Offset、Jitter、Clock Drift、硬件同步精度 |
| Level 4 | Long-term Reliability Benchmark | 24/72 小时运行、断连恢复、高负载和多传感器并发 |

## 11. 待确认的关键技术问题

以下三个问题会直接决定同步评测的实施方式：

1. **Camera 和 IMU 是否来自同一个硬件设备？** 例如一体化 Camera+IMU 模组，还是独立 USB Camera 和独立 IMU？
2. **设备是否支持硬件同步？** 例如 FSYNC、PPS、Trigger、GPIO 或共享时钟。
3. **最终应用是否用于 VIO/VSLAM？** 如果是，除了时间戳误差，还建议评测 Camera-IMU 外参、IMU 噪声参数和运动状态下的时间偏移标定。

尤其是第三点：如果目标是 VIO/VSLAM，建议把 **Camera-IMU 时间偏移标定**也纳入正式验收，而不仅仅检查两个 ROS 2 Topic 的时间戳是否一致。这样这套 Benchmark 才能真正反映传感器是否满足视觉惯性融合的要求。

---

## 参考资料

- NVIDIA Isaac ROS `correlated_timestamp_driver` 文档（isaac_ros_docs）：https://nvidia-isaac-ros.github.io/
