# mipi_cam_eval — hobot_mipi_cam 评测工程包

嵌套于 hobot_mipi_cam 目录内的独立 ament 包：C++ 接收端 recorder、Python 分析链、
测试与（后续）cases 配置。无 PLATFORM 依赖，同一套代码服务 X5/S100/S600。

架构与决策见 `../docs/评测架构定稿.md`；术语见工作区根 `CONTEXT.md`。

## 目录

```
eval/
├── src/recorder_node.cpp        # C++ recorder（CSV + 时钟旁车 .clock）
├── test/                        # 缝2(recorder)/缝3([EVAL]契约) gtest
├── test/analyze/                # 缝1(分析链) unittest（host 直接跑）
└── tools/analyze/               # parsers/metrics/report + analyze.py CLI
```

## 构建与测试（重要：嵌套包发现）

colcon 发现父包目录后不再下探，嵌套包必须显式 `--base-paths` 指向 `eval/`：

```bash
# 交叉编译（S100；容器内，build.sh 同款环境）
cd <tros_ws>
source /opt/ros/humble/setup.bash
export TARGET_ARCH=aarch64 TARGET_TRIPLE=aarch64-linux-gnu CROSS_COMPILE=/usr/bin/aarch64-linux-gnu-
export PKG_CONFIG_PATH=$(pwd)/../sysroot_docker/usr/lib/aarch64-linux-gnu/pkgconfig
colcon --log-base /tmp/colcon_log build \
  --base-paths src/box/hobot_sensor/hobot_mipi_cam/eval \
  --build-base build_eval2 --install-base install_eval2 --merge-install \
  --cmake-args --no-warn-unused-cli \
    -DCMAKE_TOOLCHAIN_FILE=$(pwd)/robot_dev_config/aarch64_toolchainfile.cmake \
    -DPLATFORM_S100=ON -DTHIRDPARTY=ON -DBUILD_TESTING=ON
```

注意：**必须从一开始就 export CROSS_COMPILE**——若首次配置时缺失，CMakeCache 会缓存
原生 gcc，后续补环境变量也无法纠正，只能删 build 目录重来。

## 跑测试

```bash
# 分析链（host，无依赖）
python3 -m unittest discover -s src/box/hobot_sensor/hobot_mipi_cam/eval/test/analyze

# C++ 测试（容器内 qemu 跑 aarch64 产物；板端直接跑亦可）
SYS=../sysroot_docker   # 建 qemu root：
mkdir -p /tmp/qroot/lib && ln -sfn $SYS/usr_s100 /tmp/qroot/usr
ln -sfn $SYS/usr_s100/lib/ld-linux-aarch64.so.1 /tmp/qroot/lib/ld-linux-aarch64.so.1
ln -sfn $SYS/usr_s100/lib/aarch64-linux-gnu /tmp/qroot/lib/aarch64-linux-gnu
export HOME=/tmp LD_LIBRARY_PATH=/opt/ros/humble/lib:<tros_ws>/install/lib
qemu-aarch64-static -L /tmp/qroot build_eval2/mipi_cam_eval/eval_log_contract_test
qemu-aarch64-static -L /tmp/qroot build_eval2/mipi_cam_eval/recorder_test
```

## recorder 用法

```bash
ros2 run mipi_cam_eval recorder --ros-args \
  -p csv_path:=/tmp/eval.csv \
  -p 'image_topics:=["/image_raw","/image_left_raw"]' \
  -p 'imu_topics:=["/imu_data"]' \
  -p 'info_topics:=["/image_raw/camera_info"]' \
  -p 'hbmem_topics:=["/hbmem_img"]'
# 产出：/tmp/eval.csv（每行一条元数据）+ /tmp/eval.csv.clock（双钟偏移，域归一用）
```

## 分析链用法

```bash
python3 tools/analyze/analyze.py --csv /tmp/eval.csv --log mipi_cam_run.log \
    --image-domain mono --out-md report.md --out-json metrics.json
```

## 板端部署待办（ticket 05）

标准全量构建（robot_dev_config/build.sh 不带 --base-paths）发现不了本嵌套包；
板端部署方案（symlink 或构建流程改造）在 ticket 05 敲定。

## 运行示例（C01 单目 / C04 双目）

用途：板端起流 → recorder 落 CSV → `[EVAL]` → 分析链。runner 仅在**仓库内**运行，
不安装到 `install/share`（需要 `eval/cases/` 与仓库 `install/setup.bash`）。

```bash
# 板端（192.168.127.10，NFS 仓库内；支持 TROS_DISTRO/PROJ/TROS_SETUP 覆盖）
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_case.sh C04 180     # 双目 3 分钟（当前主线）
./eval/run/run_case.sh C01 30      # 单目脚本保留，按需跑

# 产出：eval/logs/<CASE>_<run_label>_<时间戳>/
#   mipi_cam.log       被测节点原始 stdout/stderr（launch 加了 [mipi_cam-1] 前缀）
#   mipi_cam.eval.log  剥掉 launch 前缀后的纯 [EVAL] 行（分析链用这个）
#   launch.args        YAML params 生成的 launch 参数
#   recorder.log / recorder.csv / recorder.csv.clock / meta.txt / result.txt

# 主机（同一份 NFS 文件）
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C04_<run_label>_<时间戳>/recorder.csv \
  --log eval/logs/C04_<run_label>_<时间戳>/mipi_cam.eval.log \
  --out-md /tmp/C04_report.md --out-json /tmp/C04_metrics.json
python3 eval/tools/analyze/plot_startup.py \
  --metrics /tmp/C04_metrics.json --out /tmp/startup_timeline.png
```

runner 判绿条件：launch 退出码 ∈ {0,124,130,137,143}、CSV 非空、覆盖时长 ≥ 半分、
`.clock` 存在、`[EVAL]` 非空、YAML 中的 image topic 均有数据。`result.txt` 记录全部证据。

注意：
- 板上 `cam-service` 可能占用 sensor；停服务属于板端写操作，需先确认。
- 交叉编译的 `install/setup.bash` 会链到 `/mnt/cc_ws/tros_ws/install`，板上不存在属预期；
  `run_case.sh` 已过滤该行。
- 用例 YAML 的 `params` 会全量写成 `key:=value` 传给 launch；启动前用
  `ros2 launch ... --show-args` 校验每个 key 都被 launch 声明，未声明直接 FAIL，
  避免 `io_method` 这类参数被静默丢弃。
- `eval_log_enable` / `eval_log_events` 来自 YAML 的 `eval.log_enable` / `eval.log_events`。
- 已知观察：
  1. `/imu_data` 无数据：`/dev/iio:device1`（icm42688）为 root-only，节点以 sunrise 运行；
     已显式传 `imu_type:=icm42688`，失败会产出 `imu_error` 事件。IMU 分支再处理权限。
  2. 图像端到端偏移约 2.6s 且恒定，属 B09 域审计范围；在定性前该延迟值不可作为基线。

## 运行与交付约定（2026-11-11 起）

### 单次运行

每次 `run_case.sh` 都在 `eval/logs/` 下**新建**目录，命名
`<CASE>_<run_label>_<YYYYmmdd_HHMMSS>`，例如
`C01_single-1088x1280-nv12-30fps_20261111_165500`。
`run_label` 取自用例 YAML，用来一眼看出本轮测什么；目录不覆盖、不复用，整体 gitignored。

### 分支交付物

每个指标分支的交付物入库到 `eval/reports/<branch>/`：

| 文件 | 内容 |
|---|---|
| `summary.md` | 本轮测什么、结果、发现、结论、原始数据路径、复现命令 |
| `metrics.json` | 分析链输出的机器可读指标 |
| `plots/` | 该指标的图表（如有） |
| `review.md` | **仅当日志/分析发现问题时才有**：子代理评审结论与修复记录 |

原始日志留在 `eval/logs/<运行目录>/`，不入库；`summary.md` 里写路径。

### 评审触发条件

先看 `result.txt`、`mipi_cam.log`、`recorder.log` 和分析结果：

- **无异常**：不启动子代理评审，`summary.md` 记录「log/analysis clean, no review」。
- **有异常或语义不确定**：启动独立只读子代理，产出 `review.md` 后再交付。

### 每轮开始前

先向用户汇报：本轮测什么指标、用哪个用例/配置、准备加哪些 log/事件、预期产出、预计时长；
**得到同意后再动手**。飞书暂不写，全部本地。
