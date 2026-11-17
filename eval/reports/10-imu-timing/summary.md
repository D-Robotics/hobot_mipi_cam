# eval/10-imu-timing：IMU 采样频率/抖动/单调性

| 项 | 内容 |
|---|---|
| 分支轮次 | 10-imu-timing（`eval/07-combine` 分支逐轮提交） |
| 日期 | 2026-11-17 |
| 平台 | RDK X5 V1.0，kernel 6.1.83，TROS humble，icm42688（IIO） |
| 用例 | C16：双目 1088x1280@30 combine=1 + IMU 静置 180s，**root 运行** |
| 结果 | C-01 **202.14 Hz**（+1.07%）；C-02 p99 抖动 1.1%、max 39.5ms；C-03 倒退 0/重复 0；C-04 实测口径丢样 39/34959（**0.11%**，标称口径漏报） |
| 原始数据 | `eval/logs/C16_dual-1088x1280-nv12-30fps-combine1-imu_20000102_065153/`（60s 冒烟：`..._063606/`） |
| 指标 JSON | `metrics_C16.json` |

## 0. 前置突破：IMU 出数据的两个条件

历轮全部 `imu_topic_empty`，本轮定位并解决：

1. **权限**：`/dev/iio:device*` 为 `crw------- root root`，节点以 sunrise 运行读不到。**用 root 跑 runner**（`ssh s600`，同板 root 通道）即可，无需改板端配置。root 跑后 `warnings=` 首次为空。
2. **overlay**：icm42688 overlay 已在 `/boot/config.txt` 加载（此前 VIMS 轮修过），`iio:device1 = icm42688` 单设备布局。

## 1. 指标结果

| 指标 | 结果 | 期望（方案） | 判定 |
|---|---|---|---|
| C-01 实际频率 | **202.14 Hz**（跨度 172.9s，34959 样本） | 200Hz ±1% | 略超 +1.07%，为 ODR 晶振偏差非丢帧 |
| C-02 间隔抖动 | p50=4941µs、p99=**5054µs**（±1.1%） | p99 ≤10% 周期 | **通过**（余量大） |
| C-03 单调/重复 | 倒退 **0**、重复 **0** | 100% / 0 | **通过** |
| C-04 丢样 | 实测口径 **39 / 34959 = 0.11%** | ≤0.01% | **未达标**（11 倍超出） |

- 间隔分布高度集中（4938~4945µs 八个桶占大头），说明逐样本时戳是**驱动采样时刻**（IIO `sample.timestamp`，realtime 域），不是读取时刻。
- 驱动探针：`src=iio`（172 个窗口）、读失败 **0**、窗口内无倒退——丢样不是用户态读取失败，发生在驱动/内核侧（FIFO 或中断批量延迟）。
- 全量运行的 `pub_loss=-39` 为**修复前二进制缺终末 flush** 所致（末窗计数丢失，非发布链异常）；修复后 60s 验证 `probe_n_sum 10681 ≥ imu_rows 10652`（+29，recorder best_effort 丢样 ~0.3%，方向正确），见 `review.md`。
- **口径限制**：`rd_fail` 在 IIO 通路语义纯净（阻塞读、仅真错误）；若走 event 通道，超时也返回 -1，`rd_fail` 将含"正常无数据超时"，届时需拆分错误码。

## 2. C-04 的两处 39.5ms 大缺口

- t+4.7s 与 t+85.4s 各一次 **39.5ms**（≈8 样本）批量停顿，另有 20+ 次 9.9ms（≈2 样本）小缺口、一次 29.7ms。
- 39.5ms 停顿期间无读失败、恢复后无样本补偿（时间戳序列连续跳跃），符合**内核侧 evdev/IIO 缓冲或调度批量延迟**特征，与 04 轮图像侧 23.3s 停顿同向但量级小得多。
- **口径教训**：ODR 实际 +1.07% 时，按标称 200Hz 反推期望样本数会**漏报缺口**（标称口径 missing=0）。本轮新增**实测周期反推**口径（gap > 1.5×实测 p50 计缺口），两个口径都在报告中输出。

## 3. 埋点与修复

- 新增事件 `imu_probe`（`EVAL_EV_IMU_PROBE = 1u<<10`）：读取环 **1Hz 聚合 + 终末 flush**（200Hz 全量打点会自扰），字段 `src/n/rd_fail/ts_first/ts_last/iv_max_ns/iv_backwards`。
- `ImuManager::source_name()`：暴露当前通路（event/iio/split），本次实测为 `iio`。
- **行为修复**：`read_imu_data()` 原实现不检查 `read_sensor_data` 返回值，读失败时会把**上一帧数据重发**；现改为失败计数 + 不发布。本轮 0 次失败，行为差异未观测到，但消除了脏数据风险。
- 分析链 `imu_timing_metrics`：C-01~04 全套 + probe 对账（`pub_loss` 为窗口边界竞态近似，0.1%）；thresholds 新增 4 个 IMU 键。
- 单测 **58 项全过**（新增基础/缺口/ODR 偏差漏报/告警/缺席）。

## 4. 对后续轮次的输入

- IMU 时戳为 realtime 域（946684s 量级）；recorder 旁车 `recv−stamp` 偏移稳定性 **±8ms**（span 16.7ms）——**13-cam-imu 的域归一精度上限就是 ~±8ms**，如需亚毫秒级 Camera-IMU 配对须改驱动时戳域或提高旁车采样频率。
- `src=iio`：本板未走 event 通道（evdev），13 轮配对窗口设计按 IIO 时戳域处理。

## 5. 复现

```bash
# 板端 root（sunrise 读不到 /dev/iio*）
ssh s600 'cd /home/sunrise/workspace/hobot_mipi_cam && ./eval/run/run_case.sh C16 180'

python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C16_dual-1088x1280-nv12-30fps-combine1-imu_20000102_065153/recorder.csv \
  --log eval/logs/C16_dual-1088x1280-nv12-30fps-combine1-imu_20000102_065153/mipi_cam.eval.log \
  --out-md /tmp/C16.md --out-json /tmp/C16.json
```

## 6. 局限

- 只测 IIO 通路（src=iio）；event 通道（evdev）未在本板启用，其时戳行为未验证。
- 39.5ms 缺口的内核侧根因未定位（需使能 iio 缓冲水位/中断日志，超出零侵入评测范围）。
- C-04 0.11% 未达 0.01% 期望：属驱动/内核侧，评测如实记录。
- root 运行是权限约束（不改板端配置的前提）；若产品要求非 root，需 udev 规则（板端配置变更，另行评审）。
- 静置桌面环境，未含振动/温漂场景（C-05/06 归 11-imu-validity）。
