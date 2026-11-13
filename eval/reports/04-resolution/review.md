# eval/04-resolution 独立评审与处置

| 项 | 内容 |
|---|---|
| 评审方式 | 独立只读子代理，未修改文件 |
| 数据 | C04/C14R 首轮/C14R 复跑；见 `summary.md` |
| 总评 | 分辨率几何结论成立；C14R 首轮是采集侧 ~23.34s 停顿，不是时钟跳变；报告措辞与若干盲区已修 |

## 一、关键纠正

首轮异常不是“时间戳跳变”，而是 **`ts_mono` 上 23.34s 的采集消费停顿**：

- pipe0：`frame_id=7541 ts_mono=235754023231544` → `7547 ts_mono=235777362324659`，差 23.339s；
- pipe1：`7535→7541`，差 23.437s；
- 恢复时先吐停顿前残留旧帧（7553/7547，ts_frame 仅 +0.1s），再跳到 `8945 ts_frame=235773643711842`，所以 `ts_frame` 看似 23.2s 台阶；
- `frame_id` 差 1392 ≈ 23.2s×60fps，sensor/VIN 一直在计数，是应用侧没收到；
- 停顿窗口 EVAL 行为 0（`ts_mono` 在写日志前采集），排除日志/管道背压；`Timeout/Select/getframe failed` 均为 0；`grab failed` 约 10/s 持续 23s；
- 阻塞点应在 `getVnodeFrame` 内（getframe/invalidate/resize+memcpy/release），两 pipe 同时停指向共享资源；现有日志无法再细分。

结论：C14R 分辨率通过；首轮标注“稳定性/环境异常，不作稳定性基线”。

## 二、发现与处置

| # | 级别 | 问题 | 处置 |
|---|---|---|---|
| F1 | 高 | summary 把 7553→8945 的 ts_frame 台阶误写成“时间戳跳变” | **已修**：改为 7541→7547 的 23.34s `ts_mono` 停顿 + 残留帧解释 |
| F2 | 高 | 停顿未落到具体调用层 | **遗留**：下次复现时在 `getVnodeFrame` 加阶段打点（getframe/invalidate/copy/release），归 `02-frame-timing`/稳定性工作流 |
| F3 | 中 | meta 缺少板/宿主环境记录 | **部分修**：runner meta 增加 `board_loadavg`/`uptime_s`/`image_topics`/`info_topics`；当前运行 meta 无这些行，后续生效；宿主 NFS 负载需另记 |
| F4 | 中 | recorder best_effort，CSV 断档不能当发布断流证据 | **已说明**：summary 局限注明“连续性看 frame_in/pub_out，CSV 只看几何/接收时延” |
| F5 | 中 | `RESULT=OK` 不含稳定性，23s 停顿仍 OK | **部分修**：分析链新增 `stability_warnings`（max > 3×目标周期），报告新增“稳定性告警”；summary 首行改为“分辨率通过、首轮稳定性异常”；result.txt guardrail 留给 `02` |
| F6 | 中高 | 设备侧无 width/height 时 `all_match` 仍为 True（跨平台假通过） | **已修**：新增 `device_covered`，无设备侧数据时 `all_match=False`；S100/S600 打点遗留 |
| F7 | 中 | `_expected_for_topic` 先 combine 后 sub，`/sub_image_combine_raw` 映射错 | **已修**：先判 sub；新增 `sub_combine`/`info_sub` expected；补单测 |
| F8 | 中 | 整 topic 缺失不计 mismatch；combine camera_info 未校验 | **部分修**：`resolution_metrics` 新增 `expected_topics`/`missing_topics`；C04/C14R 的 info_topics 已补 combine 两路；**C04 回归验证已校验 combine left/right camera_info（1088x1280，mismatch 0）**；C14R combine info 待下次复跑 |
| F9 | 低 | “旋转宽高正确交换”未隔离验证内容方向 | **已修措辞**：改为“输出尺寸与配置一致”；rotation=0/90 方向对照遗留 |
| F10 | 低 | `grab failed` 无 topic/无节流 | **遗留**：归 `02-frame-timing` 观测指标 |
| F11 | 低 | `sync_task` combine `get_empty_buff` 可能空指针 | **已修**：主/辅 combine 均判空 + sleep + continue |
| F12 | 低 | 引用了不存在的 `review.md` | **已修**：本文件落盘 |

## 三、单测

新增/更新：

- `device_covered=False` + missing topic → `all_match=False`；
- `sub_combine`/`info_sub` 期望；
- `stability_warnings`（max > 3×周期）；
- 原有 resolution/expected 用例保留。
- 当前共 **23 项单测通过**。

## 四、建议

1. **判定**：C04、C14R（首轮+复跑）分辨率通过；首轮稳定性异常不作基线。
2. **归口**：23.34s 停顿与 guardrail/大 gap 归 `02-frame-timing`；根因深挖走独立 VIMS/grab-failed 工作流，不阻塞 04。
3. **复跑**：建议 C14R ≥3–5 次、C04 3 次对照，记录宿主机负载/NFS 状态，估计发生率；单次未复现不足以定性。
4. **下次复现看**：frame_in 相邻 `ts_mono` 差、gap 前后 fid/ts_frame、`grab failed` 起止/频率、Timeout/Select/getframe 计数、板端 meminfo/loadavg、dmesg、recorder 断档与退出耗时、getVnodeFrame 阶段打点。
5. **边界用例**：sub_stream+combine 的 `/sub_image_combine_raw`、整 topic 缺失、只错 combine、rotation=0/90 对照、设备侧缺打点。
