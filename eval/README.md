# mipi_cam_eval — hobot_mipi_cam 评测工程包

嵌套于 hobot_mipi_cam 目录内的独立 ament 包：C++ 接收端 recorder、Python 分析链、
用例配置、板端 runner/suite 编排器与专项工具。无 PLATFORM 依赖，同一套代码服务
X5/S100/S600（`exposure` 等事件目前仅 X5 实现埋点）。

架构与决策见 `../docs/评测架构定稿.md`；总方案见 `../docs/X5评测总体方案.md`；
**全部评测结论见 `reports/00-overall/summary.md`（总报告）**。

## 目录

```
eval/
├── src/recorder_node.cpp        # C++ recorder（CSV + 时钟旁车 .clock；支持 hbmem 订阅）
├── cases/                       # 用例 YAML（params/thresholds/recorder 订阅/load 负载）
├── launch/                      # 评测专用 launch（single/dual，参数全量声明）
├── run/
│   ├── run_case.sh              # 单用例 runner（读 YAML → 起 recorder+被测节点 → result.txt）
│   ├── run_suite.py             # suite 编排器（多用例 → 逐个分析 → 三态汇总报告）
│   ├── suites.yaml              # suite 预设（smoke/core/latency/resolution/full）
│   ├── run_bw.sh                # 带宽：跑用例同时并行采样 ros2 topic bw
│   ├── run_res.sh               # 长测：跑用例 + 5s 周期资源采样（CPU/RSS/fd/温度）
│   ├── run_startstop.sh         # 启停 N 次稳定性（进程/设备残留检查）
│   └── run_stab8h.sh            # 8h 长稳（数据落 /tmp，心跳+资源采样）
├── test/                        # C++ gtest（[EVAL] 契约 / recorder）
├── test/analyze/                # 分析链单测（host 直接跑，74 项）
├── tools/analyze/               # parsers/metrics/report/thresholds/suite_report + analyze.py
└── reports/                     # 逐轮报告 + 00-overall 总报告（交付物）
```

## 快速开始（换 sensor / 改配置后的标准入口）

```bash
# 板端（NFS 仓库内）。IMU 用例需 root（/dev/iio* root-only）→ ssh s600（root 通道）
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_suite.py --suite smoke      # C04 60s，链路自检
./eval/run/run_suite.py --suite core       # C04/C04S/C07/C08 核心基线 ~12min
./eval/run/run_suite.py --suite latency    # C04/C07/C04B/C06 延迟专题
./eval/run/run_suite.py C04 C07 --duration 60   # 任意组合/覆盖时长

# 产出：eval/logs/suite_<名>_<时间戳>/suite_report.md（PASS/WARN/FAIL 三态汇总
#       + 帧率/丢帧/分辨率/配对/延迟总表）+ suite_metrics.json；任一 FAIL 退出码非 0
./eval/run/run_suite.py --analyze-only <运行目录...>   # 只重算汇总（不跑板端）
```

新 sensor 流程：复制一份 cases/C04.yaml 改 params（channel/分辨率/帧率）→
`--suite smoke` 冒烟 → `--suite core` 出基线 → 按需跑 `05`（曝光/触发行为需重标定，
t0 域偏移逐 boot 变化）。阈值默认值在 `tools/analyze/thresholds.py`，用例 YAML 的
`thresholds:` 可覆盖（如 `pair_diff_p99_ms: 4.0`）。

## 单用例 runner

```bash
./eval/run/run_case.sh <CASE> [DURATION] [OUTDIR]    # 时长覆盖 YAML；OUTDIR 可指 /tmp
```

- 用例 YAML 的 `params` 全量传 launch（`--show-args` 校验，未声明参数直接 FAIL）；
- `eval.log_events` 选择 `[EVAL]` 事件类别（startup/frame_in/frame_drop/queue_stat/
  exposure/sync_pair/combine_out/pub_out/imu_probe/imu_stat/imu_error）；
- `recorder.{image,imu,info,hbmem}_topics` 空列表自动省略传参（rcl 空数组会崩）；
- `load.cpu_affinity`/`load.cmd` 正向负载（如 `taskset -c 0 yes`）；
- 判绿写 `result.txt`：launch 退出码 ∈ {0,124,130,137,143}、CSV 覆盖 ≥ 半程、
  `.clock` 存在、`[EVAL]` 非空、必需 topic 有数据；
- 运行目录 `eval/logs/<CASE>_<run_label>_<时间戳>/`（gitignored，不复用）。

## 用例矩阵

| 用例 | 配置 | 覆盖 |
|---|---|---|
| C01/C04 | 单目/双目 1088x1280@30 nv12 combine=1 | 基线（C04 为 suite 基准） |
| C04S | C04 + CPU0 负载 60s | 丢帧正向对照 |
| C04B | C04 + bgr8 | 转换开销/延迟（已知：bgr8+combine 不可用） |
| C05/C06 | combine=2 / combine=1+辅流 | 拼接模式/满配 |
| C07/C08 | +lpwm / +realtime | 触发 t0 / 域审计（C08 受 P1 溢出影响） |
| C11/C12B/C12C | 负载+订阅 / 2 订阅 / 4 订阅 | E-06/07 |
| C14R | 1280x1088@10 r90 | 旋转/分辨率 |
| C16 | C04 + IMU 静置（root） | C-01~06（imu_probe/imu_stat） |
| C19 | C06 满配 28800s | 8h 长稳（配合 run_stab8h.sh） |
| CH1/CH2/CH3 | io_method=shared_mem（combine=1 / +lpwm / combine=0） | hbmem 通路 |
| HI1/HI2 | 延迟×CPU 负载（IMU+图像 / 仅 IMU） | 走 run_latency.sh，见专项工具 |

## 专项工具

```bash
./eval/run/run_bw.sh C06 180          # 稳态窗口并行采样 7 topic 带宽 → bw_*.txt
                                      # 注意：bw 对 hbmem 按定长 6.22MB 计（×2.97 虚高）
./eval/run/run_res.sh C06 1800        # 长测 + 资源采样 → res_samples.csv（分析链自动读）
./eval/run/run_startstop.sh 8 C04 30  # 启停 8 次，残留检查 → startstop_*/
nohup ./eval/run/run_stab8h.sh 28800 &  # 8h 长稳：数据落 /tmp/stab8h_*，心跳+资源
```

### IMU/Image 延迟 × CPU 负载（封装仓库根 `imu_latency_test.py`）

```bash
./eval/run/run_latency.sh HI1          # IMU+图像，预热15+空载60+压力60+恢复60（root：ssh s600）
./eval/run/run_latency.sh HI2          # 仅 IMU（严格 IMU 判定）
./eval/run/run_suite.py --suite imu-latency   # 一键跑 HI1+HI2 + 三态汇总

python3 eval/tools/analyze/latency_csv.py --run-dir <运行目录>   # 分段统计+判据
# 脚本 v2：IMU 用独立进程订阅（与图像彻底隔离）；预热 --warmup 丢弃 IIO 缓冲积压
#   （实测积压约 9s，用例配 15）；汇总表分空载/压力/恢复；写 <stem>.phases.json
#   供分析精确分段；image 值剥离 ts_mono−ts_frame 的 boot 偏移（本轮达 371ms）
# 结论（2026-11-26）：IMU 采样→接收 3.0~3.8ms（p99 4.1），3 核压力无影响；
#   图像 18.5/24.7ms，压力期 p95 1.0×；两例 PASS
```

长测（>10min）必须 nohup + done 标志轮询（单 ssh 上限 600s）：
`nohup bash -c "./eval/run/run_res.sh C06 1800 > /tmp/r.log 2>&1; touch /tmp/r.done" &`

## 分析链

```bash
# 单运行（自动读同目录 meta.txt 的期望值/阈值、bw_*.txt、res_samples.csv）
python3 eval/tools/analyze/analyze.py --csv <运行目录>/recorder.csv \
  --log <运行目录>/mipi_cam.eval.log --out-md r.md --out-json r.json

python3 -m unittest discover -s eval/test/analyze   # 74 项单测
```

- 30min 数据 ~13s（大日志性能已修：避免热路径重建列表/线性扫描）；
- 8h 数据 ~4min、内存 ~3GB——在**主机**跑（板端 3GB 内存不够）；
- 报告章节自动按事件启用（startup/帧时序/丢帧/分辨率/曝光/配对/拼接/延迟分解/
  IMU 时序与有效性/Cam-IMU 配对与漂移/接收对账与延迟/带宽/资源稳态）。

## 构建与测试（嵌套包发现）

colcon 发现父包后不再下探，必须显式 `--base-paths` 指向 `eval/`（X5 已验证形态）：

```bash
# 交叉编译（docker pc_tros_ubuntu22.04:v1.0.0，详见根 CLAUDE.md 的 X5 模板）
colcon build --base-paths eval --build-base build_eval --install-base install \
  --merge-install --cmake-args -DCMAKE_TOOLCHAIN_FILE=<tros_ws>/...cmake \
  -DPLATFORM_X5=ON -DTHIRDPARTY=ON -DBUILD_TESTING=OFF
# recorder 含 hbmem 订阅需 hbm_img_msgs（tros_ws/install 已有，CMake QUIET 探测）
```

## 已知坑位（都踩过，改前先读）

1. **bash 脚本禁用 `set -u`**：humble `setup.bash` 引用未定义变量会被 -u 静默终止。
2. **recorder 空 topic 数组不传参**（runner 已处理）：rcl 对 `:=[]` 参数类型推断崩溃。
3. **IMU 必须 root**（`/dev/iio:device*` root-only）；非 root 静默无数据（有 imu_error 事件）。
4. **8h 级日志在板端 /tmp**（eMMC 43G）；CSV+日志 ~1.2GB，勿落 NFS 仓库。
5. `ros2 topic bw` 对 hbmem 失真（定长消息口径）；hbmem 带宽用元数据估算。
6. 老板时钟回到 2000 年：目录名字典序不代表新旧，取"最新"用 mtime/快照差集。

## 运行与交付约定

- 单次运行目录 `eval/logs/<CASE>_<run_label>_<时间戳>/` 不入库；分支交付物
  `eval/reports/<轮次>/`：summary.md（必）、metrics.json（必）、review.md（有问题/
  自评审时）、plots/（如有）。**报告入口：`reports/00-overall/summary.md`**。
- 每轮开始先汇报"测什么/加哪些 log"，同意后动手；飞书暂不写，全部本地。
- 评测只记录证据不改产品逻辑；修复走独立分支（已知待修：realtime int32 溢出 P1、
  trig_valid 假阳性 P2，见总报告问题清单）。
