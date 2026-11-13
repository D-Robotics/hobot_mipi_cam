# eval/01-startup 独立评审与处置

| 项 | 内容 |
|---|---|
| 评审方式 | 独立只读子代理，未修改文件 |
| 评审对象 | startup 打点、分析链、报告、C04 用例、runner 参数映射 |
| 数据 | `eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000103_163356/` |
| 总评 | 主链路（detect/gdc/create_vflow/sensor_fixed/vflow_start/first_frame/first_pub）位置正确，数值与报告一致，零侵入成立；3 项重要问题已处置 |

## 发现与处置

| # | 级别 | 问题 | 处置 |
|---|---|---|---|
| F1 | 重要 | `calib` 窗口只覆盖 backend 缓存查询，真实 EEPROM 在公共 `MipiCam::init`，被算进约 73ms 间隙 | **已修正 summary**：`calib` 标注为缓存查询，73ms 标为“公共 init（含 EEPROM）”；公共路径单独打点**遗留**到后续分支 |
| F2 | 重要 | 单目分支 `detect_sensors/sensor_fixed/gdc_cfg/init_pre_vflow` 未打点，C01 注释“startup 已就绪”不成立 | **已修正 C01 注释**为“单目子阶段待补”；本轮只验双目，单目打点**遗留**到单目轮次 |
| F3 | 重要 | `io_method` 未被 launch 声明，YAML 参数被静默丢弃；`eval_log_enable` 被硬编码 | **已修**：单/双 launch 增加 `io_method`；runner 从 YAML 读 `eval.log_enable`；新增 `ros2 launch --show-args` 参数覆盖校验，未声明直接 FAIL |
| F4 | 建议 | `first_pub` 受订阅者门控，缺失 topic 时分析链静默取剩余最小值 | **已修**：summary 注明口径；分析链新增 `missing_first_pub` 告警 |
| F5 | 建议 | 报告不显示 `ret`，init 失败时时间线看似正常 | **已修**：报告增加 `ret` 列；`startup_metrics` 增加 `complete`，失败/不完整显式告警 |
| F6 | 建议 | 契约写 `dur_ms`，实现与分析链都没有 | **已修**：`startup_metrics` 增加 `stage_durations`（begin/end 配对），报告新增“阶段耗时”表 |
| F7 | 建议 | `std::vector<bool>` 位压缩，当前无竞争但有隐患 | **遗留**：当前 main/sub 分属不同 vector 且单线程写；下轮改 `vector<char>` |
| F8 | 建议 | t0 是构造函数体起点，不是对象构造起点 | **已修**：报告与 summary 文案改为“MipiCamNode 构造函数体起点” |
| F9 | 建议 | 单测未覆盖回退/失败/去重；Python 测试未接入 CTest | **已修**：新增 t0 缺失、不完整、去重、缺失 first_pub、报告不完整用例，16 项全过；CTest 接入**遗留** |
| F10 | 建议 | 缺 t0 时绘图把阶段画到 0ms | **已修**：缺 `since_t0_ms` 跳过；全部缺失直接报错 |
| F11 | 建议 | summary 三处表述与事实不符 | **已修**：耗时表改为“起止打点相减 + 间隙单列”；calib/EEPROM 修正；评审文件已落盘 |
| F12 | 建议 | 安装版 `run_case.sh` 找不到仓库目录 | **已修**：从 `eval/CMakeLists.txt` 移除 runner 安装；runner 仅在仓库内运行 |

## 评审确认的 PASS 项

- 阶段打点包住真实调用；`first_frame` 在 `getVnodeFrame` 成功/memcpy 后；`first_pub` 在 `publish()` 返回后；ROS/hbmem 双路径覆盖。
- `t0` 与 `since_t0_ms` 自洽；first_frame/first_pub 去重取最早；JSON 结构合理。
- 零侵入成立：默认关闭零输出；热路径仅一次原子读/一次性 bool。
- 关机 `hb_mem_free_buf` 报错与本轮埋点无关，归 `eval/20-start-stop`。

## 最终数据

- `complete=true`，`missing_first_pub=[]`。
- 首帧 5029.912ms；GDC 2578.378ms；create_vflow 661.069+654.288ms；detect_sensors 609.521ms；sensor_fixed_host 152.586+152.764ms；vflow_start 3.404+3.044ms。
- `stage_durations` 已写入 `metrics.json`，报告表格与 summary 一致。
