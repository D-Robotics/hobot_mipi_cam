# eval/01-startup：双目首帧启动时间

| 项 | 内容 |
|---|---|
| 分支轮次 | `eval/01-startup` |
| 日期 | 2026-11-11 |
| 平台 | RDK X5 V1.0，kernel 6.1.83，TROS humble |
| 传感器 | sc132gs 双目（pipe0/pipe1） |
| 用例 | C04：双目 1088x1280@30 nv12 / ros / sensor ts / `dual_combine=1` |
| 时长 | 180s（有效覆盖 172.84s） |
| 结果 | `RESULT=OK`（launch rc=124 为到时中断，属预期） |
| 原始数据 | `eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000103_165330/` |
| 图表 | `plots/startup_timeline.png` |
| 指标 JSON | `metrics.json` |

## 1. 结论

- **首帧启动时间（MipiCamNode 构造函数体起点 → 第一帧）：5029.9ms**；首次发布 5052.1ms。
- 飞书初始期望为 ≤2s；本轮为**基线模式，仅统计不判定**。实测明显高于初始期望。
- 主要耗时（阶段起止打点相减；相邻事件之间的间隙单列）：

| 阶段 | 耗时 | 占首帧 | 证据 |
|---|---:|---:|---|
| **GDC 配置生成**（`gdc_cfg`） | **2578.4ms** | 51.3% | `gdc_cfg_begin/end` |
| `create_and_run_vflow` ×2 | 1315.4ms | 26.2% | pipe1 661.1 + pipe0 654.3 |
| 传感器探测（`detect_sensors`） | 609.5ms | 12.1% | `detect_sensors_begin/end` |
| `vp_sensor_fixed_mipi_host_1` ×2 | 305.4ms | 6.1% | pipe1 152.6 + pipe0 152.8 |
| 公共 init（含 EEPROM 标定）→ backend 入口 | 73.0ms | 1.5% | 间隙：`backend_init_begin`→`backend_init_enter`；日志 `i2c bus: 6, EEPROM FLAG: UNION` 落在此段 |
| backend 入口 → 开始探测传感器 | 40.1ms | 0.8% | 间隙：`backend_init_enter`→`detect_sensors_begin` |
| `vflow_start` ×2 | 6.5ms | 0.1% | `vflow_start_begin/end` |
| 启动到首帧等待 | 49.1ms | 1.0% | `backend_start_end`→`first_frame` |
| 其余（构造/节点初始化等） | ~52.5ms | 1.0% | 相邻事件间隙 |

> 说明：backend 内的 `calib` 窗口只有 0.087ms，覆盖的是已构造单例的缓存查询；
> 真正的 EEPROM 扫描发生在公共 `MipiCam::init`，落在上表“公共 init→backend 入口”的 73.0ms 间隙内。
> 本轮未在该公共路径单独打点，属于已知口径缺口。
>
> **GDC 是首位开销**。C04 使用 `gdc_enable=true`；若要压缩启动时间，优先考虑 GDC bin 缓存/预生成或 `gdc_enable=false` 场景对比。

## 2. 启动时间线（相对 t0）

t0（MipiCamNode 构造函数体起点，monotonic）= `172266430597387 ns`

| 相对 t0 (ms) | 阶段 | pipe | 说明 |
|---:|---|---|---|
| 23.047 | node_init | - | eval 配置完成、参数声明前 |
| 26.373 | node_init_call_begin | - | `MipiCamNode::init()` 入口 |
| 99.445 | backend_init_enter | - | 进入 X5 后端 `init()` |
| 139.589 | detect_sensors_begin | - | 开始探测双目 |
| 749.110 | detect_sensors_end | - | 探到 2 路 |
| 749.178 → 901.764 | sensor_fixed_host pipe=1 | 1 | 152.6ms |
| 901.774 → 901.861 | calib（backend 缓存查询） | - | 0.087ms；真实 EEPROM 在公共 init，见 §1 |
| 901.868 → 3480.247 | gdc_cfg | - | **2578.4ms** |
| 3480.297 → 4141.366 | create_vflow pipe=1 | 1 | 661.1ms |
| 4141.402 → 4294.166 | sensor_fixed_host pipe=0 | 0 | 152.8ms |
| 4294.178 → 4948.466 | create_vflow pipe=0 | 0 | 654.3ms |
| 4973.773 → 4980.248 | vflow_start pipe=0/1 | 0,1 | 6.5ms |
| 4981.828 | node_init_call_end | - | 节点初始化完成 |
| 5029.912 | first_frame | 0 | 首帧 |
| 5034.925 | first_frame | 1 | 第二路首帧 |
| 5052.057 | first_pub | - | `/image_left_raw`、`/image_right_raw` |
| 5064.550 | first_pub | - | `/image_combine_raw` |

## 3. 直接证据说明

- 每个阶段都在真实调用前后打点：`vp_sensor_detect_2` 循环、`vp_sensor_fixed_mipi_host_1`、`getCalibrationParams`、`gen_gdc_bin_stereo`、`create_and_run_vflow`、`hbn_vflow_start`。
- `first_frame` 在 `hbn_vnode_getframe` 成功、`memcpy` 完成后打点，每 pipe/流只打一次。
- `first_pub` 在 `publish()` 返回后打点，每个 publisher 只打一次（本次 ROS 通路，recorder 有订阅）。
- 阶段耗时由 `begin/end` 配对自动产出（`metrics.json.stage_durations`），不是人工相减。
- 未使用 `frame_id` 缺口、CPU 占用等旁证去反推启动耗时。

## 4. 本轮顺带观察（不归 startup 结论）

| 现象 | 证据 | 归属 |
|---|---|---|
| 右目比左目/拼接时间戳晚约 3.07–3.12ms | 多组左目最近右目差 3.072–3.118ms；拼接与右目配对率 0 | `eval/06-stereo-sync`（C04 `lpwm=false`，后续补 lpwm 对比） |
| 接收端比 `pub_out` 少约 1.3%–1.7% | 左 5186→5116、右 5187→5100、拼接 5186→5117 | `eval/15-recv-drop` |
| 关机阶段 3 次 `grab failed` + 2 次 `hb_mem_free_buf: Memory module is not found` | 日志最后 15 行 | `eval/20-start-stop` |
| `/imu_data` 无数据 | `/dev/iio:device1` root-only；`imu_error` 可见 | IMU 分支（权限/root） |

## 5. 评审

独立只读子代理已评审：主链路正确、数值一致、零侵入成立；3 项重要问题（`calib` 归属、单目分支漏打点、`io_method` 静默丢弃）已处置，`io_method` 与参数覆盖校验已修复，单目/公共 init 打点遗留到后续分支。详见 `review.md`。

## 6. 复现

```bash
# 板端（192.168.127.10，NFS 仓库）
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_case.sh C04 180

# 主机分析
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000103_165330/recorder.csv \
  --log eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000103_165330/mipi_cam.eval.log \
  --out-md /tmp/C04_report.md --out-json /tmp/C04_metrics.json
python3 eval/tools/analyze/plot_startup.py \
  --metrics eval/reports/01-startup/metrics.json \
  --out eval/reports/01-startup/plots/startup_timeline.png
```

## 7. 局限

- t0 = `MipiCamNode` 构造函数体起点，**不含基类构造 / 进程拉起 / `ros2 launch` 启动开销**；若按“launch 到首帧”口径，数值会更大。
- 仅测 `dual_combine=1` 主码流；`sub_stream`、`single`、`gdc_enable=false`、hbmem 通路未在本轮覆盖。
- 单目分支的 `detect_sensors / sensor_fixed_host / gdc_cfg / init_pre_vflow` 尚未打点（C01 脚本保留，但 startup 子阶段不完整）。
- 公共 `MipiCam::init` 的 EEPROM 标定未单独打点，当前归在 73.0ms 间隙内。
- `first_pub` 依赖有订阅者（本轮 recorder 在订阅）；无订阅者时不会触发，分析链会对缺失 topic 告警。
