# eval/08-device-latency 独立评审与处置

| 项 | 内容 |
|---|---|
| 评审方式 | 独立只读子代理，未修改文件 |
| 数据 | C04 / C07 / C04B / C06；见 `summary.md` |
| 总评 | 核心数值可从原始日志复算；「就绪→出队」的语义、t0 系统性偏差与 topic 映射需修，已全部处置 |

## 一、评审发现与处置

| # | 级别 | 问题 | 处置 |
|---|---|---|---|
| 1 | 中 | 「就绪→出队」并非纯队列等待：`deq_mono` 在 `getImage` 返回后，段内含 getFrame 的 0~10ms 轮询、两次 RCLCPP_INFO、拷贝/转换 | **已修**：字段改名为 `ready_to_get_ns`，报告列名改为「就绪→取帧返回」，并在口径与 summary 中写明 10ms 轮询与转换 |
| 2 | 中 | `offset=median(ts_hw−曝光打点)` 含硬件→软件打点延迟 D1，t0 系统性偏晚 D1，t0 延迟是下界；offset 未导出 | **已修**：metrics 导出 `t0_offset_p1/p50/p99_ns`，报告标注偏移与半宽；summary 写明 D1≈0.66ms、t0 延迟为下界、±1.5ms 不确定度 |
| 3 | 中 | `_latency_source` 未命中 left/right 时默认 pipe=1，`/hbmem_img`、`/image_raw` 等会错配 | **已修**：改为显式 topic→(pipe,stream) 表（含单目/hbmem/sub hbmem）；未登记 topic 标 `unmapped`，报告告警；就绪匹配为 0 也告警 |
| 4 | 低 | C04B 5189−4295=894 ≠ recycled 888，残差未说明 | **已修**：summary 给出残差 6 的末窗解释，并用 `empty_null=0`、`fq_full=0` 排除缓冲耗尽/限流 |
| 5 | 低 | NV12 体积按 2B/px 算错（主 2.7MB、拼接 5.6MB） | **已修**：改为 1.5B/px → 2.09MB / 4.18MB / 0.52MB |
| 6 | 低 | “≈3.5×”只贴合右目 | **已修**：改为左 3.83×、右 3.48×、拼接 7.90× |
| 7 | 低 | C07 拼接 +13.4ms 归因是推断 | **已修**：标注“推断（本轮无 combine_out）” |
| 8 | 低 | “~3.9s 域偏移”是旧 run 数字 | **已修**：改为本次 boot 实测 C04≈37.8ms、C07≈116.6ms，并注明按 boot 变化 |
| 9 | 低 | 接收侧左目写 3915/5188，实际 3932 | **已修**：改为 3932/5188（24.2%） |
| 10 | 低 | 单测缺负值、combine 无 combine_out、单目/hbmem 映射、重复键 | **已修**：新增 3 组用例，共 47 项；报告补相应告警 |
| 11 | 低 | 就绪定义不对称（frame_in 在拷贝后、combine_out 在拼接拷贝前） | **已修**：在口径中注明，跨流比较需考虑 |
| 12 | 低 | ready/t0 字典覆盖写静默丢样本 | **已修**：增加 `ready_duplicates`/`t0_duplicates` 并在报告告警 |

## 二、复核结论

- nv12 左/右目 10.6~11.6 ms、拼接 15.5~16 ms、辅流 7.4~9.6 ms 可复算。
- C07 t0 口径 2.87/3.89 ms（采集）、13.33/14.58 ms（发布）、拼接 26.72 ms，与原始日志一致；带 ~0.66ms 下界偏差与 ±1.5ms 归一不确定度。
- C04B：combine_out 5189（30.00fps）、pub 4295（24.84fps）、recycled 888，自洽；bgr8+combine 不可用的结论成立。

## 三、当前测试

- `python3 -m unittest discover -s eval/test/analyze`：**47 项全过**。
- `git diff --check`、`py_compile` 通过。
