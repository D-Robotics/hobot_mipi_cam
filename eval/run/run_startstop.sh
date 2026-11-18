#!/usr/bin/env bash
# 20-start-stop：启停 N 次（默认 8），每次检查退出码与进程残留。
# 产出：eval/logs/startstop_<ts>/summary.txt（逐次 RESULT 与残留检查）
# 用法：./run_startstop.sh [N] [CASE] [EACH_S]
set -o pipefail
N=${1:-8}
CASE=${2:-C04}
EACH=${3:-30}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJ=$(cd "$SCRIPT_DIR/../.." && pwd)
OUT="$PROJ/eval/logs/startstop_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
echo "case=$CASE n=$N each=${EACH}s" > "$OUT/summary.txt"
OK=0
for i in $(seq 1 "$N"); do
  "$SCRIPT_DIR/run_case.sh" "$CASE" "$EACH" > "$OUT/run_$i.log" 2>&1
  RC=$?
  # 进程残留：runner 结束后 2s 仍存在的 mipi_cam / recorder
  sleep 2
  RES=$(pgrep -x mipi_cam | wc -l)
  RES2=$(pgrep -x recorder | wc -l)
  # 设备占用：isp 设备节点仍在（存在即正常，无 fuser 占用为佳）
  BUSY=$(fuser /dev/isp0_cap 2>/dev/null | wc -w)
  LINE="run=$i rc=$RC proc_mipi_cam=$RES proc_recorder=$RES2 isp_busy=$BUSY"
  if [ "$RC" -eq 0 ] && [ "$RES" -eq 0 ] && [ "$RES2" -eq 0 ]; then
    LINE="$LINE verdict=OK"; OK=$((OK+1))
  else
    LINE="$LINE verdict=FAIL"
  fi
  echo "$LINE" | tee -a "$OUT/summary.txt"
done
echo "success=$OK/$N" | tee -a "$OUT/summary.txt"
[ "$OK" -eq "$N" ]
