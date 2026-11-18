# eval/16-recv-latency：接收延迟（E-04）

| 项 | 内容 |
|---|---|
| 分支轮次 | 16-recv-latency（`eval/07-combine` 分支逐轮提交） |
| 日期 | 2026-11-18 |
| 用例 | 复用 C07（lpwm，可信 t0，180s，root）——零新埋点、纯分析侧 |
| 结果 | DDS 段（pub→recv）：left/right **1.2~1.3ms**、combine **2.1ms**（p99 <3ms）；t0→接收端到端：left **14.1ms**、right 15.3ms、combine 28.2ms；与 08 轮设备侧分段自洽 |
| 原始数据 | `eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000101_083935/` |
| 指标 JSON | `metrics_C07.json` |

## 1. 结果

| topic | DDS 段 p50/p95/p99(ms) | t0→recv p50/p95(ms) |
|---|---|---|
| /image_left_raw（2.1MB） | **1.22** / 1.99 / 2.66 | **14.12** / 19.82 |
| /image_right_raw（2.1MB） | **1.29** / 2.59 / 2.94 | 15.30 / 19.78 |
| /image_combine_raw（4.2MB） | **2.06** / 2.63 / 2.97 | **28.19** / 35.82 |

- **DDS 段与消息体积线性**：2.1MB→1.2ms、4.2MB→2.1ms（≈2GB/s 的共享内存传输+序列化开销），无负值。
- **端到端分段自洽**（与 08 轮设备侧对账）：
  - left：13.33（t0→pub，08 轮）+ 1.22（DDS）≈ **14.1** ✅
  - combine：26.72 + 2.06 ≈ **28.2** ✅
  - 全链路每一毫秒都有归属，无未解释段。

## 2. 完整延迟图谱（X5，1088×1280@30 nv12，lpwm）

```
t0(曝光中点) ──2.9ms──> frame_in ──~7.4ms──> pub_out ──1.2ms──> 订阅回调（left）
   │                        │                    │
   └── combine：t0 ──26.7ms──> pub ──2.1ms──> 订阅回调
       （含等待另一路 + 10ms 轮询 + 拼接拷贝）
```

## 3. 分析链

- 新增 `recv_latency_metrics`：DDS 段（同 stamp 配对 recv−pub）+ E-04 端到端（trig 可信时 recv−t0，13 轮同法归一）；无可信 t0 时只出 DDS 段，不混入帧 ts 近似。
- 单测 **70 项全过**（新增 DDS/E-04 合成、无 pub 场景；修正了 fixture 的 trig 与 exp 不自洽问题）。

## 4. 局限

- 仅 lpwm 配置有可信 t0；普通配置的端到端需域归一（±8ms 旁车不确定度）后另算。
- DDS 段含 recorder 进程的调度与回调排队（单订阅者、best_effort）；多订阅者下会增大。
- 30fps/1088×1280 单一组合；bgr8（8.4MB）的 DDS 段未单独测。

## 5. 复现

```bash
python3 eval/tools/analyze/analyze.py \
  --csv eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000101_083935/recorder.csv \
  --log eval/logs/C07_dual-1088x1280-nv12-30fps-combine1-lpwm_20000101_083935/mipi_cam.eval.log \
  --out-md /tmp/C07l.md --out-json /tmp/C07l.json
```
