# eval/16-recv-latency 评审与处置

| 项 | 内容 |
|---|---|
| 评审方式 | 主会话自评审（子代理 Bash 持续被权限拦截，同前例） |
| 数据 | C07 180s（复用 13 轮运行） |

## 一、自评审检查点

| # | 检查点 | 结论 |
|---|---|---|
| 1 | DDS 段配对口径 | 同 stamp 精确匹配（pub_out.ts_frame ∈ CSV stamps）；15 轮已证 stamp 无重复（pub_duplicates=0），配对无歧义 |
| 2 | t0 归一与 08/13 轮一致性 | 同一 offset 法、同一代码形态；端到端减法自洽（14.1=13.33+1.22、28.2=26.72+2.06）即交叉验证 |
| 3 | 实现过程中误留的占位 import（mipi_cam_latency_src） | 已在提交前删除并 grep 确认；py_compile/单测通过 |
| 4 | 单位 | dds_ns 以 ns 存、报告 `_fmt_ns` 除 1e6 显示 ms——渲染正确；调试打印曾标错单位（µs 误标 ms），不影响交付物 |
| 5 | fixture 修正 | trig 初始设为 hw−5ms 与 exp=10ms 不自洽（真实数据 tv−trig=exp），已改 hw−exp；顺带修 join 缺失 |
| 6 | summary 数字 | 与分析输出一致；DDS 与体积的线性关系（2.1MB→1.2ms）与共享内存传输量级相符 |

## 二、当前测试

- `python3 -m unittest discover -s eval/test/analyze`：**70 项全过**。
- `git diff --check`、`py_compile` 通过。

## 三、遗留

- 非 lpwm 配置的端到端（需旁车归一，精度 ±8ms）。
- 多订阅者/bgr8 的 DDS 段（扩展期）。
