# eval/19+20：资源稳态与启停（门槛轮，30min 长测）

| 项 | 内容 |
|---|---|
| 分支轮次 | 19-resources + 20-start-stop（合并，`eval/07-combine` 逐轮提交） |
| 日期 | 2026-11-18 |
| 用例 | C06 满配 1800s（资源采样 355 点 @5s）；C04 启停 8 次×30s |
| 结果 | **门槛全过**：RSS 稳态零增长（150216KB 恒定）、fd 恒 133、帧率 30min 恒 30.000 **零丢帧**；启停 **8/8**，无进程/设备残留 |
| 原始数据 | 19：`eval/logs/C06_dual-1088x1280-nv12-30fps-combine1-sub_20000101_140519/`（含 res_samples.csv）<br>20：`eval/logs/startstop_20000101_135811/` |
| 指标 JSON | `metrics_C06_res.json` |

## 1. 19 资源稳态（30min，门槛判定）

| 项 | 实测 | 判据 | 判定 |
|---|---|---|---|
| RSS | 稳态 150216KB **恒定**（max 153.5MB 在启动期） | 30min 无单调增长（≤5%） | ✅ 增长 0.0 |
| fd | 133 → 133 | 无增长 | ✅ 增长 0 |
| 帧率/丢帧 | 30.000fps 恒定，**0 丢帧**（主+辅 4 流） | 无中断 | ✅ |
| CPU | p50=100%、p95=120%（单进程忙循环+发布） | 记录 | 记录 |
| 温度 | 57~78°C（thermal zone0） | 记录，降频未发生（帧率不崩佐证） | ✅ |

- **fd 伪增长教训**：首采样点 17（建链前 5s）、末点 33（退出收尾中）；直接取端点会误判 +16 泄漏。分析链已改为**稳态窗口**（去头去尾 2%）判定——启动建链与退出释放不参与门槛。
- fd 曲线：t+5s 一次建到 133，**29 分钟纹丝不动**。
- CPU 恒 1 核是采集/发布线程的 1ms 轮询忙等特征（代码 `usleep(1*1000)`），30min 无爬升。

## 2. 20 启停稳定性（8 次）

```
run=1..8 rc=0 proc_mipi_cam=0 proc_recorder=0 isp_busy=0 verdict=OK
success=8/8
```

- 8 次启停全部 RESULT=OK；每次结束后 2s **无 mipi_cam/recorder 残留进程**、**无 /dev/isp0_cap 占用**——sensor 释放干净。
- 工具：`eval/run/run_startstop.sh [N] [CASE] [EACH_S]`。

## 3. 工具与分析链

- `eval/run/run_res.sh <CASE> <DURATION>`：跑长用例 + 5s 周期采样 /proc（CPU 累计/RSS/fd/温度）到 run 目录 `res_samples.csv`；30min 用例以 nohup 后台跑（单 ssh 上限 10min）。
- `res_metrics`：稳态窗口判定（RSS 增长/fd 增长/CPU 分位/温度），报告含门槛告警。
- 单测 72 项全过（资源解析以合成数据验证）。

## 4. 局限

- 30min 短稳 ≠ 8h/24h 长稳（方案已裁剪）；慢泄漏（小时级）不可见。
- 温度仅 thermal zone0，未逐核；降频未直接读 cpufreq（以帧率不崩间接佐证）。
- 启停间隔 2s 检查点较粗；瞬时残留（<2s）不可见。
