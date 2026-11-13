# eval/04-resolution：分辨率准确性（基础 + 旋转）

| 项 | 内容 |
|---|---|
| 分支轮次 | `eval/04-resolution` |
| 日期 | 2026-11-12 |
| 平台 | RDK X5 V1.0，kernel 6.1.83，TROS humble |
| 传感器 | sc132gs 双目（pipe0/pipe1） |
| 基础用例 | C04：双目 1088x1280@30 nv12 / `dual_combine=1`，180s |
| 旋转用例 | C14R：双目 1280x1088@10 nv12 / `rotation=90°` / `dual_combine=1`，180s（首轮 + 复跑） |
| 结果 | C04/C14R 分辨率通过（全部运行 mismatch 0）；C14R 首轮另有约 23.34s 采集停顿（偶发，复跑未复现）；C04 修复后回归验证通过（含 combine camera_info） |
| 原始数据 | C04：`eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_102603/`<br>C14R 首轮：`eval/logs/C14R_dual-1280x1088-nv12-10fps-combine1-r90_20000104_102927/`<br>C14R 复跑：`eval/logs/C14R_dual-1280x1088-nv12-10fps-combine1-r90_20000104_113335/`<br>C04 回归：`eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_115500/` |
| 指标 JSON | `metrics.json`（C04 + C14R） |

## 1. C04 基础：全部一致

| 面 | 流/topic | 期望 | 实际（帧数） | mismatch |
|---|---|---|---|---|
| device | pipe0/main | 1088x1280 | 1088x1280(5184) | 0 |
| device | pipe1/main | 1088x1280 | 1088x1280(5184) | 0 |
| published | `/image_left_raw` | 1088x1280 nv12 | 1088x1280 nv12(5142) | 0 |
| published | `/image_right_raw` | 1088x1280 nv12 | 1088x1280 nv12(5152) | 0 |
| published | `/image_combine_raw` | 1088x2560 nv12 | 1088x2560 nv12(5115) | 0 |
| published | left/right `camera_info` | 1088x1280 camera_info | 1088x1280 camera_info | 0 |

- **修复后回归验证**（新二进制 + combine 两路 camera_info + meta topic 清单）：设备侧 1088x1280(5186/5185)，左右目 1088x1280，combine 1088x2560，left/right/combine 四路 camera_info 均 1088x1280，mismatch 0，`missing_topics` 0，无稳定性告警。

## 2. C14R 旋转：输出尺寸与配置一致

| 面 | 流/topic | 期望 | 实际（帧数） | mismatch |
|---|---|---|---|---|
| device | pipe0/main | 1280x1088 | 1280x1088(1499) | 0 |
| device | pipe1/main | 1280x1088 | 1280x1088(1496) | 0 |
| published | `/image_left_raw` | 1280x1088 nv12 | 1280x1088 nv12(1146) | 0 |
| published | `/image_right_raw` | 1280x1088 nv12 | 1280x1088 nv12(1148) | 0 |
| published | `/image_combine_raw` | 1280x2176 nv12 | 1280x2176 nv12(1165) | 0 |
| published | left/right `camera_info` | 1280x1088 camera_info | 1280x1088 camera_info | 0 |

- 输出尺寸与配置一致：1280x1088、combine 纵向拼接 1280x2176。
- 注意：本轮只验证“尺寸与配置一致”，**未做旋转内容方向/画面方向的独立验证**（需单独的 rotation=0/90 对照或方向标记图）。
- 设备侧与发布侧、camera_info 全部一致；**分辨率准确性 100%**。
- **复跑 C14R2 同样全绿**：设备侧 1280x1088（1728/1727 帧），combine 1280x2176，camera_info 1280x1088，mismatch 0；10.0fps，间隔 max 100.01/100.00ms，无回退/重复。
- combine 的两路 camera_info 本轮未订阅（recorder 只订了 left/right）；用例已补入，后续轮次校验。

## 3. C14R 独立稳定性异常（非分辨率失败）

C14R 首轮出现一次约 23.34s 的**采集消费停顿**（不是时钟跳变）：

- 真实停顿在 `ts_mono` 上：pipe0 `frame_id=7541 ts_mono=235754023231544` → `7547 ts_mono=235777362324659`，差 **23.339s**；pipe1 `7535→7541` 差 **23.437s**；两 pipe 同时停、同时恢复。
- 恢复时先吐出停顿前残留的旧帧（7553/7547，ts_frame 仅比 7541 晚约 0.1s），随后跳到 `frame_id=8945 ts_frame=235773643711842`；所以 `ts_frame` 看似跳了 23.2s，实际是停顿期间 VSE 输出的旧帧 + 最新帧。`frame_id` 差 1392 ≈ 23.2s×60fps，说明 sensor/VIN 一直在计数，是应用侧没收到。
- 停顿窗口内 EVAL 行为 **0**（`ts_mono` 在写日志前采集），排除日志/管道背压；`Timeout occurred` / `Select error` / `hbn_vnode_getframe` 失败均为 0；`grab failed` 约 10/s 持续 23s，发布线程活着但取不到帧。
- 评审定位：阻塞在 `getVnodeFrame` 内部（`hbn_vnode_getframe` / `hb_mem_invalidate_buf_with_vaddr` / `resize+memcpy` / `releaseframe`），两 pipe 同时停指向共享资源（hb_mem/VPF 全局锁）；现有日志无法再细分，需下次复现时加阶段打点。
- 首轮同期 recorder 侧有 5 段 >0.5s 接收断档（合计约 60s，最大 27.9s），C04 与 C14R2 均为 0；首轮退出拖延 23–36s。更像板/宿主机（NFS）异常窗口，而非 rotation 配置的必然结果。
- **复跑 C14R2 未复现**：10.0fps、max 100.01/100.00ms、无回退/重复、无 23.34s 停顿，`grab failed` 仅收尾 3 次。

结论：C14R 分辨率通过；首轮应标注“稳定性/环境异常，不作稳定性基线”。该问题归 `02-frame-timing`（大 gap/guardrail）与独立稳定性工作流，不阻塞 04 分辨率分支。

## 4. 评审

独立只读子代理结论：C14R 首轮是**采集消费侧 ~23.34s 停顿**，不是时钟跳变/发布阻塞；分辨率结论不受影响，但报告措辞、设备侧覆盖、sub_combine 映射、整 topic 缺失、combine camera_info 等需修正。处置见 `review.md`。

## 5. 复现

```bash
# 板端（192.168.127.10，NFS 仓库）
cd /home/sunrise/workspace/hobot_mipi_cam
./eval/run/run_case.sh C04 180
./eval/run/run_case.sh C14R 180

# 主机分析（自动从 meta.txt 读取期望值）
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_102603/recorder.csv \
  --log eval/logs/C04_dual-1088x1280-nv12-30fps-combine1_20000104_102603/mipi_cam.eval.log \
  --out-md /tmp/C04_res.md --out-json /tmp/C04_res.json
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C14R_dual-1280x1088-nv12-10fps-combine1-r90_20000104_102927/recorder.csv \
  --log eval/logs/C14R_dual-1280x1088-nv12-10fps-combine1-r90_20000104_102927/mipi_cam.eval.log \
  --out-md /tmp/C14R_res.md --out-json /tmp/C14R_res.json
```

## 6. 局限

- C14R 使用产品 r90 参考配置的 1280x1088 / 10fps；未覆盖 640x544、640x352 等其余分辨率/旋转矩阵。
- 仅测 `dual_combine=1`；`sub_stream`、`single`、hbmem 通路未在本轮覆盖。
- C14R 首轮的单次 23.34s 停顿未复现；需更多复跑估计发生率，归 `02-frame-timing`/稳定性工作流。
- 旋转只验证输出尺寸，未验证画面方向；需 rotation=0/90 对照或方向标记图。
- S100/S600 的 `frame_in` 尚无 width/height；`resolution_metrics` 已用 `device_covered` 防静默假通过，但跨平台设备侧覆盖待补。
- recorder 为 best_effort + KeepLast(1)，CSV 会丢消息；连续性以 `frame_in`/`pub_out` 为准，CSV 只用于载荷几何与接收侧时延。
- C04 回归验证已校验 combine 两路 camera_info（1088x1280，mismatch 0）；C14R 的 combine info 待下次带新用例的复跑。
