# eval/10-imu-timing 评审与处置

| 项 | 内容 |
|---|---|
| 评审方式 | 独立子代理再次被权限系统拦停（Auto mode unavailable）；按 08 轮先例改主会话自评审，覆盖原评审清单的关键点 |
| 数据 | C16 全量 180s + 60s 冒烟×2；见 `summary.md` |

## 一、自评审检查点与处置

| # | 检查点 | 结论与处置 |
|---|---|---|
| 1 | 1Hz 聚合窗口语义、flush 竞态、`pub_loss=-39` 反向差 | **发现真缺陷**：循环退出无终末 flush，最后一窗口计数丢失（-39 即末窗样本数）。**已修**：循环后补 `imu_probe_flush`；重编后 60s 验证 `probe_n_sum=10681 ≥ imu_rows=10652`（+29，recorder best_effort 丢样 ~0.3%，方向正确）。全量 180s 运行使用修复前二进制，其 probe 对账差 39 为末窗缺失，C-01~04 结论不受影响（CSV 样本序列完整） |
| 2 | 读失败不发布的行为风险；event 通道超时返回 -1 会污染 rd_fail | IIO 通路 `read()` 阻塞、仅真错误返回 -1，rd_fail=0 语义纯净。**口径限制**：若走 event 通道（evdev），`kReadTimeoutMs` 超时也返回 -1，rd_fail 将含"正常无数据超时"——已在 summary 注明；event 通路启用时需拆分错误码 |
| 3 | `source_name()` 对非 icm 传感器返回 "iio" | 标签性偏差：新 sensor（非 icm42688）会被标 "iio"。无功能影响，记录为限制 |
| 4 | C-04 实测周期反推的算术与 ODR 偏差单测 | 复算自洽：39.53ms/4941µs→int(8.00)-1=7；总 missing_gap=39=2×7+5+2+2+…+1×N；ODR+1% 单测证明标称口径漏报（missing_est=0）而实测口径抓到 8 缺失 |
| 5 | gap_events 截前 10 条 | 20+ 次 9.9ms 小缺口只展示 5 条（报告）；完整计数在 missing_gap。可接受 |
| 6 | summary 数字与原始数据一致性 | 202.14Hz / p99 5054µs / 34959 / 39 缺失 / ±8ms（span 16.7ms）均与分析输出和自算一致 |
| 7 | thresholds `imu_interval_p99_ratio=0.5` 与告警公式 `abs(p99−标称周期)>tol` | 语义自洽：tol=0.5×周期=2.5ms，p99 5054µs 偏离 5000µs 仅 54µs 不告警；若 ODR 整体偏 1%（p50 4941），p99≈4954 偏离 46µs 也不告警——公式对"抖动"敏感、对"整体平移"不敏感，符合 C-02 本意（抖动），频率平移由 C-01 比率抓 |

## 二、当前测试

- `python3 -m unittest discover -s eval/test/analyze`：**58 项全过**。
- `git diff --check`、`py_compile`、交叉编译（含终末 flush）均通过；60s 板端验证通过。

## 三、遗留

- 39.5ms 内核侧缺口根因（需驱动侧日志，超出零侵入范围）。
- event 通道的 rd_fail 口径（该通路启用时处理）。
- 非根运行的 udev 方案（板端配置变更，另行评审）。
