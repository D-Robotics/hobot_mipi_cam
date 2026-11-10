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
