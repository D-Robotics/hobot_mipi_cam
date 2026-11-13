# eval/00-foundation 冒烟报告

| 项 | 内容 |
|---|---|
| 分支 | `eval/00-foundation`（基于 `feature-eval`，工作区改动未提交） |
| 日期 | 2026-11-11 |
| 平台 | RDK X5 V1.0，kernel 6.1.83，TROS humble |
| 传感器 | sc132gs 单目（1088x1280），`/dev/iio:device1` = icm42688 |
| 用例 | C01：单目主码流 1088x1280@30 nv12 / ros / sensor ts，20s |
| 结论 | **P0 通过**：构建闭环、起流、recorder CSV、`[EVAL]`、分析链全部打通 |

## 1. 构建

| 包 | 结果 | 产物 |
|---|---|---|
| `mipi_cam` | 成功（1m08s） | `install/lib/libmipi_cam_plugin.so`（ARM aarch64） |
| `mipi_cam_eval` | 成功（41.9s） | `install/lib/mipi_cam_eval/recorder`（aarch64）、`install/share/mipi_cam_eval/{launch,cases,run}` |

- 环境：docker `pc_tros_ubuntu22.04:v1.0.0`，sysroot `~/cc_ws_x5_s100/sysroot_docker/usr_x5`，`PLATFORM_X5=ON`。
- 无 `[ERROR]`；仅既有 CMake 变量未使用类告警。
- 属主已 chown 回 `1000:1000`。

## 2. 板端运行（C01，20s）

```
RESULT=OK
launch_rc=124            # timeout 到时，属预期
csv_rows=489             # /image_raw
csv_cover_s=16.26
eval_lines=491           # 490 pub_out + 1 imu_error
clock_ok=1
clock_jump_suspected=0
topic_counts=/image_raw=489
warnings=imu_topic_empty:/imu_data info_topic_empty:/image_raw/camera_info
```

- 无 mipi_cam / recorder / ros2 launch 残留；节点日志 `process has finished cleanly`，dmesg 有 sensor release。
- 原始数据：`eval/logs/C01_single-1088x1280-nv12-30fps_20000103_140813/`（gitignored）。

## 3. 分析链结果

| 指标 | 实测 |
|---|---|
| `/image_raw` 帧数 | 489 |
| 均值 fps | 30.019 |
| 接收间隔 p50 / p99 / max | 33.88 / 34.37 / 34.90 ms |
| stamp 回退 / 重复 | 0 / 0 |
| 设备侧 `pub_out` | 490 |
| 端到端 `recv − frame_ts` p50 / p99 | 2616.64 / 2622.25 ms |

## 4. 已知观察（不阻塞 P0）

1. **IMU 未出数（权限）**：`/dev/iio:device1` 为 `crw------- root root`，节点以 `sunrise` 运行，
   初始化失败并产出 `[EVAL] event=imu_error err=imu_init_fail detail=type_icm42688`。
   `/imu_data` 0 行属预期。IMU 分支将以 root 运行或申请 udev 权限（后者属板端写操作，需确认）。
2. **图像端到端偏移约 2616ms 且恒定**：接收端 DDS 延迟仅 ~3ms，偏移在设备侧时间戳/排队；
   留 `eval/09-domain-audit` 定性。**在 B09 结论前，该值不得作为延迟基线。**
3. **启动器收尾**：节点日志显示 clean exit，但 20s 用例 runner 实测 elapsed≈38s，`timeout` 返回 124；
   无残留、sensor 正常释放。启停耗时归 `eval/20-start-stop` 量化。
4. **camera_info 无数据**：`camera_calibration_file_path=default` 且无 EEPROM 标定；E-10 需真实标定文件。

## 5. 评审与修正

独立只读子代理评审结论：无阻断项；6 项重要问题已在交付前修复：

1. `RESULT=OK` 判据加严：launch rc、覆盖时长、`.clock`、必需 topic、`result.txt`。
2. OUTDIR 非空拒绝、`mkdir`/recorder 失败显式失败、数据新鲜度。
3. recorder 就绪轮询（替代固定 `sleep 2`）。
4. `trap` INT/TERM 立即退出；`setsid -w` + PGID 精确收尾；PID 置空。
5. eval launch 显式传 `imu_type:=icm42688`，IMU 失败可见。
6. runner 读取 YAML 的 `duration_s / log_events / recorder topics`，并校验用例存在。

未在本分支处理：`.clock` 周期采样（当前仅启动单点 + `clock_jump_suspected` 墙钟检测）、
多用例 launch 参数映射（留给 `run_suite.py`）、camera_info 标定数据。

## 6. 复现

```bash
# 构建（开发机 docker，见仓库 CLAUDE.md X5 humble 模板）
# 板端
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_case.sh C01 20

# 主机分析
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C01_<时间戳>/recorder.csv \
  --log eval/logs/C01_<时间戳>/mipi_cam.eval.log \
  --out-md /tmp/C01_report.md --out-json /tmp/C01_metrics.json
```
