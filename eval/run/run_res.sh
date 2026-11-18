#!/usr/bin/env bash
# 19-resources：跑长用例并采样被测节点资源（CPU/RSS/fd/温度），全程只读。
# 产出：<run 目录>/res_samples.csv（ts,pid,cpu_utime_s,cpu_stime_s,rss_kb,fd,thermal）
# 用法：./run_res.sh <CASE> <DURATION_S>（例：./run_res.sh C06 1800）
set -o pipefail
CASE=${1:-C06}
DURATION=${2:-1800}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJ=$(cd "$SCRIPT_DIR/../.." && pwd)
source /opt/tros/humble/setup.bash >/dev/null 2>&1
source "$PROJ/install/setup.bash" 2>/dev/null

"$SCRIPT_DIR/run_case.sh" "$CASE" "$DURATION" &
RUNNER=$!
# 等 mipi_cam 进程出现（启动 ~10s 内）
PID=""
for _ in $(seq 1 60); do
  PID=$(pgrep -x mipi_cam | head -1)
  [ -n "$PID" ] && break
  sleep 2
done
[ -n "$PID" ] || { echo "no mipi_cam pid"; kill $RUNNER 2>/dev/null; exit 2; }
echo "pid=$PID"

D=$(ls -dt "$PROJ"/eval/logs/"${CASE}"_* 2>/dev/null | head -1)
OUT="$D/res_samples.csv"
echo "ts,pid,utime_s,stime_s,rss_kb,fd,thermal_c" > "$OUT"
THERM=$(ls /sys/class/thermal/thermal_zone*/temp 2>/dev/null | head -1)
START=$(date +%s)
while kill -0 "$PID" 2>/dev/null; do
  U=$(awk '{print $14}' /proc/$PID/stat 2>/dev/null)
  S=$(awk '{print $15}' /proc/$PID/stat 2>/dev/null)
  R=$(awk '/VmRSS/{print $2}' /proc/$PID/status 2>/dev/null)
  F=$(ls /proc/$PID/fd 2>/dev/null | wc -l)
  T=""
  [ -n "$THERM" ] && T=$(( $(cat "$THERM" 2>/dev/null || echo 0) / 1000 ))
  [ -n "$U" ] && echo "$(date +%s),$PID,$((U/100)),$((S/100)),${R:-0},${F:-0},${T:-}" >> "$OUT"
  sleep 5
done
wait $RUNNER
echo "RUNNER_RC=$?"
wc -l "$OUT"
