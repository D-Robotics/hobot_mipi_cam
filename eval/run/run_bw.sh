#!/usr/bin/env bash
# 17-bandwidth：跑指定用例并在稳态窗口用 `ros2 topic bw` 采样每 topic 带宽。
# 产出：run 目录下 bw_<topic 安全名>.txt（ros2 topic bw 的平均输出）
# 用法：./run_bw.sh <CASE> [DURATION]（默认 C06 180）
# 注意：不能用 set -u——humble 的 setup.bash 引用未定义变量会被 -u 直接退出
set -o pipefail
CASE=${1:-C06}
DURATION=${2:-180}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJ=$(cd "$SCRIPT_DIR/../.." && pwd)
TROS_DISTRO=${TROS_DISTRO:-humble}
# shellcheck disable=SC1091
source /opt/tros/${TROS_DISTRO}/setup.bash >/dev/null 2>&1
source "$PROJ/install/setup.bash" 2>/dev/null

# 后台起 runner（正常写 eval/logs/<CASE>_...）
"$SCRIPT_DIR/run_case.sh" "$CASE" "$DURATION" &
RUNNER_PID=$!
sleep 45   # 等稳态（启动 ~5s + 缓冲），并定位最新 run 目录
D=$(ls -dt "$PROJ"/eval/logs/"${CASE}"_* 2>/dev/null | head -1)
[ -n "$D" ] || { echo "no_run_dir"; kill "$RUNNER_PID" 2>/dev/null; exit 2; }
echo "outdir=$D"

for TOPIC in /image_left_raw /image_right_raw /image_combine_raw \
             /sub_image_left_raw /sub_image_right_raw /sub_image_combine_raw /imu_data; do
  SAFE=${TOPIC//\//_}
  ( timeout 30 ros2 topic bw "$TOPIC" > "$D/bw_${SAFE}.txt" 2>/dev/null ) &
done
wait $(jobs -p | grep -v "$RUNNER_PID") 2>/dev/null
wait "$RUNNER_PID"
echo "RUNNER_RC=$?"
ls -l "$D"/bw_*.txt 2>/dev/null | head -8
