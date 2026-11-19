#!/usr/bin/env bash
# 8h 长稳：跑 C19（满配 28800s），OUTDIR 固定 /tmp（数据仅分析用，不入仓库）。
# 产出（均在 /tmp/stab8h_<ts>/）：
#   run_case 全套（recorder.csv / mipi_cam.eval.log / result.txt ...）
#   res_samples.csv   资源采样（15s 周期）
#   heartbeat.txt     每 10min 健康快照（近似 fps / RSS / fd / 温度）
# 用法：nohup ./run_stab8h.sh [DURATION_S] &   （默认 28800）
set -o pipefail
DURATION=${1:-28800}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJ=$(cd "$SCRIPT_DIR/../.." && pwd)
OUT="/tmp/stab8h_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
echo "$OUT" > /tmp/stab8h.current

"$SCRIPT_DIR/run_case.sh" C19 "$DURATION" "$OUT" &
RUNNER=$!
PID=""
for _ in $(seq 1 60); do
  PID=$(pgrep -x mipi_cam | head -1)
  [ -n "$PID" ] && break
  sleep 2
done
[ -n "$PID" ] || { echo "no pid" >> "$OUT/heartbeat.txt"; exit 2; }

echo "ts,pid,utime_s,stime_s,rss_kb,fd,thermal_c" > "$OUT/res_samples.csv"
THERM=$(ls /sys/class/thermal/thermal_zone*/temp 2>/dev/null | head -1)
while kill -0 "$PID" 2>/dev/null; do
  U=$(awk '{print $14}' /proc/$PID/stat 2>/dev/null)
  S=$(awk '{print $15}' /proc/$PID/stat 2>/dev/null)
  R=$(awk '/VmRSS/{print $2}' /proc/$PID/status 2>/dev/null)
  F=$(ls /proc/$PID/fd 2>/dev/null | wc -l)
  T=""; [ -n "$THERM" ] && T=$(( $(cat "$THERM" 2>/dev/null || echo 0) / 1000 ))
  [ -n "$U" ] && echo "$(date +%s),$PID,$((U/100)),$((S/100)),${R:-0},${F:-0},${T:-}" >> "$OUT/res_samples.csv"
  # heartbeat：整 10min 窗口一行，fps 用 eval.log 尾部 1MB 的 frame_in 计数近似
  if [ $(( $(date +%s) % 600 )) -lt 15 ]; then
    FPS=$(tail -c 1000000 "$OUT/mipi_cam.eval.log" 2>/dev/null | grep -c "event=frame_in")
    ERR=$(grep -c "event=frame_drop" "$OUT/mipi_cam.eval.log" 2>/dev/null)
    echo "$(date +%s) fps_tail1MB=$FPS drop_total=$ERR rss=${R:-0} fd=${F:-0} t=${T:-}" >> "$OUT/heartbeat.txt"
  fi
  sleep 5
done
wait $RUNNER
echo "RUNNER_RC=$?" >> "$OUT/heartbeat.txt"
