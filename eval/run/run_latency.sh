#!/usr/bin/env bash
# IMU/Image 延迟 × CPU 负载 runner（封装仓库根 imu_latency_test.py，脚本内容不改）
#
# 用法：
#   ./run_latency.sh [CASE] [TOTAL_S] [OUTDIR]
#   ssh s600 'cd /home/sunrise/workspace/hobot_mipi_cam && ./eval/run/run_latency.sh HI1'
#
# 与 run_case.sh 的差异：不起 recorder，被测订阅端就是 imu_latency_test.py
# （脚本自身出 CSV/图/统计，避免第二个订阅者抢 CPU 污染测量）。
#
# 产出（OUTDIR 下，默认 eval/logs/<CASE>_<run_label>_<ts>/）：
#   latency_imu.csv / latency_image.csv / latency_plot_{imu,image}.png  ← 脚本产出
#   mipi_cam.log / mipi_cam.eval.log / launch.args / meta.txt / result.txt

set -o pipefail

CASE=${1:-HI1}
TOTAL_ARG=${2:-}
OUTDIR_ARG=${3:-}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null && pwd)
PROJ=${PROJ:-$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd)}
TROS_DISTRO=${TROS_DISTRO:-humble}
TROS_SETUP=${TROS_SETUP:-/opt/tros/${TROS_DISTRO}/setup.bash}
LATENCY_PY="$PROJ/imu_latency_test.py"

die() { echo "RESULT=FAIL reason=$1" >&2; exit "${2:-1}"; }

[ -f "$LATENCY_PY" ] || die "latency_script_missing path=$LATENCY_PY" 2
YAML_PATH="$PROJ/eval/cases/${CASE}.yaml"
[ -f "$YAML_PATH" ] || die "case_yaml_missing path=$YAML_PATH" 2
[ -f "$TROS_SETUP" ] || die "tros_setup_missing path=$TROS_SETUP" 2

# ---- 读用例 YAML ----
YV_TMP=$(mktemp) || die mktemp_failed 2
ARGS_TMP=$(mktemp) || die mktemp_failed 2
if ! python3 - "$YAML_PATH" "$ARGS_TMP" > "$YV_TMP" <<'PY'
import json
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as f:
    d = yaml.safe_load(f) or {}
ev = d.get('eval') or {}
il = d.get('imu_latency') or {}
events = ev.get('log_events', ['all'])
if isinstance(events, list):
    events = ','.join(str(x) for x in events)
pre = float(il.get('pre_stress', 60))
stress = float(il.get('stress_duration', 60))
post = float(il.get('post_stress', 60))
print(int(pre + stress + post))
print(events)
print(d.get('run_label') or d.get('name') or '')
print(d.get('launch') or '')
print(il.get('imu_topic', '/imu_data'))
print(json.dumps(il.get('image_topics') or []))
print(il.get('image_domain', 'mono'))
print(pre)
print(stress)
print(post)
print(int(il.get('stress_cores') or max((__import__('os').cpu_count() or 2) - 1, 1)))
print(json.dumps(d.get('thresholds') or {}))
print('True' if ev.get('log_enable', True) else 'False')
with open(sys.argv[2], 'w', encoding='utf-8') as af:
    for k, v in (d.get('params') or {}).items():
        if isinstance(v, bool):
            s = 'True' if v else 'False'
        elif isinstance(v, float):
            s = repr(v)
        else:
            s = str(v)
        af.write(f"{k}:={s}\n")
PY
then
  rm -f "$YV_TMP" "$ARGS_TMP"
  die case_yaml_parse_failed 2
fi
mapfile -t YV < "$YV_TMP"
rm -f "$YV_TMP"
PHASE_TOTAL=${YV[0]:-180}
EVAL_LOG_EVENTS=${YV[1]:-all}
RUN_LABEL=${YV[2]:-}
EVAL_LAUNCH=${YV[3]:-}
IMU_TOPIC=${YV[4]:-/imu_data}
IMAGE_TOPICS_JSON=${YV[5]:-[]}
IMAGE_DOMAIN=${YV[6]:-mono}
PRE_STRESS=${YV[7]:-60}
STRESS_DUR=${YV[8]:-60}
POST_STRESS=${YV[9]:-60}
STRESS_CORES=${YV[10]:-3}
THRESHOLDS_JSON=${YV[11]:-}
EVAL_LOG_ENABLE=${YV[12]:-True}
[ -n "$THRESHOLDS_JSON" ] || THRESHOLDS_JSON='{}'
[ -n "$EVAL_LAUNCH" ] || die launch_not_set 2
[ -f "$PROJ/eval/launch/$EVAL_LAUNCH" ] || die "launch_missing path=$PROJ/eval/launch/$EVAL_LAUNCH" 2

TOTAL=${TOTAL_ARG:-$PHASE_TOTAL}
case "$TOTAL" in ''|*[!0-9]*) die "invalid_duration value=$TOTAL" 2 ;; esac

SAFE_LABEL=$(printf '%s' "$RUN_LABEL" | tr -c 'A-Za-z0-9._-' '_' | sed 's/__*/_/g; s/^_//; s/_$//')
RUN_PREFIX="$CASE"; [ -n "$SAFE_LABEL" ] && RUN_PREFIX="${CASE}_${SAFE_LABEL}"
OUTDIR=${OUTDIR_ARG:-$PROJ/eval/logs/${RUN_PREFIX}_$(date +%Y%m%d_%H%M%S)}
if [ -e "$OUTDIR" ] && [ -n "$(find "$OUTDIR" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
  die "outdir_not_empty path=$OUTDIR" 2
fi
mkdir -p "$OUTDIR" || die outdir_create_failed 2
mv "$ARGS_TMP" "$OUTDIR/launch.args" || die launch_args_write_failed 2
LAUNCH_ARGS_FILE="$OUTDIR/launch.args"

# ---- 环境 ----
# shellcheck disable=SC1090
source "$TROS_SETUP" >/dev/null 2>&1 || die "tros_setup_source_failed path=$TROS_SETUP" 2
[ -f "$PROJ/install/setup.bash" ] || die "missing_install_setup path=$PROJ/install/setup.bash" 2
SETUP_ERR=$(mktemp) || die mktemp_failed 2
# shellcheck disable=SC1091
source "$PROJ/install/setup.bash" 2>"$SETUP_ERR"
grep -v '^not found: "/mnt/cc_ws/' "$SETUP_ERR" >&2 || true
rm -f "$SETUP_ERR"

PKG_PREFIX=$(ros2 pkg prefix mipi_cam 2>/dev/null || true)
[ -n "$PKG_PREFIX" ] || die package_mipi_cam_not_found 3

# YAML params 必须被 launch 声明（防静默丢参）
SHOW_ARGS=$(ros2 launch mipi_cam_eval "$EVAL_LAUNCH" --show-args 2>/dev/null || true)
if [ -n "$SHOW_ARGS" ]; then
  UNSUPPORTED=""
  while IFS= read -r line; do
    key=${line%%:=*}; [ -n "$key" ] || continue
    printf '%s\n' "$SHOW_ARGS" | grep -q "^    '${key}':" || UNSUPPORTED="$UNSUPPORTED $key"
  done < "$LAUNCH_ARGS_FILE"
  [ -n "$UNSUPPORTED" ] && die "launch_args_unsupported:$UNSUPPORTED" 2
fi

{
  echo "case=$CASE"
  echo "run_label=$RUN_LABEL"
  echo "stage=imu-latency-cpu"
  echo "launch=$EVAL_LAUNCH"
  echo "launch_args=$(tr '\n' ' ' < "$LAUNCH_ARGS_FILE" 2>/dev/null)"
  echo "phase_pre_s=$PRE_STRESS"
  echo "phase_stress_s=$STRESS_DUR"
  echo "phase_post_s=$POST_STRESS"
  echo "stress_cores=$STRESS_CORES"
  echo "image_domain=$IMAGE_DOMAIN"
  echo "imu_topic=$IMU_TOPIC"
  echo "image_topics=$IMAGE_TOPICS_JSON"
  echo "thresholds=$THRESHOLDS_JSON"
  echo "eval_log_events=$EVAL_LOG_EVENTS"
  echo "outdir=$OUTDIR"
  echo "run_start=$(date -Iseconds)"
  echo "kernel=$(uname -r)"
  echo "board_loadavg=$(cut -d' ' -f1-3 /proc/loadavg 2>/dev/null)"
  echo "git_commit=$(git -C "$PROJ" rev-parse --short HEAD 2>/dev/null || echo unknown)"
} > "$OUTDIR/meta.txt"

LAUNCH_PID=""; LAUNCH_PGID=""
cleanup() {
  [ -n "$LAUNCH_PGID" ] && kill -INT -"$LAUNCH_PGID" 2>/dev/null || true
  [ -n "$LAUNCH_PID" ] && kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# ---- 1) 被测节点（launch 时长 = 三阶段 + 20s 余量）----
LAUNCH_DUR=$((TOTAL + 20))
export LAUNCH_PIDFILE="$OUTDIR/launch.pgid" LAUNCH_DUR EVAL_LAUNCH LAUNCH_ARGS_FILE
# 内层 setsid bash 是独立进程，未导出的变量取不到（曾导致 eval_log_enable:= 空值）
export EVAL_LOG_ENABLE EVAL_LOG_EVENTS
setsid -w bash -c '
  echo $$ > "$LAUNCH_PIDFILE"
  mapfile -t args < "$LAUNCH_ARGS_FILE"
  exec timeout --signal=INT --kill-after=15 "$LAUNCH_DUR" \
    ros2 launch mipi_cam_eval "$EVAL_LAUNCH" "${args[@]}" \
    eval_log_enable:="$EVAL_LOG_ENABLE" \
    eval_log_events:="$EVAL_LOG_EVENTS" \
    log_level:=warn
' > "$OUTDIR/mipi_cam.log" 2>&1 &
LAUNCH_PID=$!
for _ in $(seq 1 50); do [ -s "$OUTDIR/launch.pgid" ] && break; sleep 0.05; done
[ -s "$OUTDIR/launch.pgid" ] && LAUNCH_PGID=$(cat "$OUTDIR/launch.pgid")

# 等节点起流（进程 + 首帧 EVAL 行，最多 30s）
READY=0
for _ in $(seq 1 60); do
  if pgrep -x mipi_cam >/dev/null 2>&1 && grep -q "event=frame_in" "$OUTDIR/mipi_cam.log" 2>/dev/null; then
    READY=1; break
  fi
  sleep 0.5
done
if [ "$READY" -ne 1 ]; then
  {
    echo "RESULT=FAIL reason=node_not_ready"
    echo "launch_log_tail:"
    tail -5 "$OUTDIR/mipi_cam.log" 2>/dev/null
  } > "$OUTDIR/result.txt"
  die node_not_ready 6
fi
sleep 3   # 让采集/发布进入稳态后再开始计时

# ---- 2) 延迟测量（脚本自带三阶段与压力）----
IMG_CSV_ARG=$(printf '%s' "$IMAGE_TOPICS_JSON" | python3 -c 'import json,sys; print(",".join(json.load(sys.stdin)))')
python3 "$LATENCY_PY" \
  --imu_topic "$IMU_TOPIC" \
  --image_topics "$IMG_CSV_ARG" \
  --image_domain "$IMAGE_DOMAIN" \
  --pre_stress "$PRE_STRESS" --stress_duration "$STRESS_DUR" --post_stress "$POST_STRESS" \
  --stress_cores "$STRESS_CORES" \
  --output "$OUTDIR/latency.csv" \
  --plot "$OUTDIR/latency_plot.png" \
  > "$OUTDIR/latency_test.log" 2>&1
LAT_RC=$?

# ---- 3) 收尾 ----
[ -n "$LAUNCH_PGID" ] && kill -INT -"$LAUNCH_PGID" 2>/dev/null || true
if [ -n "$LAUNCH_PID" ]; then wait "$LAUNCH_PID" 2>/dev/null; fi
LAUNCH_RC=$?
LAUNCH_PID=""; LAUNCH_PGID=""

# 剥 launch 前缀，生成纯 [EVAL] 行
grep '\[EVAL\] ts_mono=' "$OUTDIR/mipi_cam.log" 2>/dev/null \
  | sed 's/^\[mipi_cam-[0-9]*\] //' > "$OUTDIR/mipi_cam.eval.log"
EVAL_LINES=$(wc -l < "$OUTDIR/mipi_cam.eval.log" 2>/dev/null || echo 0)

# ---- 4) 判绿 ----
IMU_ROWS=0; IMG_ROWS=0
[ -f "$OUTDIR/latency_imu.csv" ] && IMU_ROWS=$(($(wc -l < "$OUTDIR/latency_imu.csv") - 1))
[ -f "$OUTDIR/latency_image.csv" ] && IMG_ROWS=$(($(wc -l < "$OUTDIR/latency_image.csv") - 1))
{
  echo "case=$CASE"
  echo "outdir=$OUTDIR"
  echo "latency_rc=$LAT_RC"
  echo "launch_rc=$LAUNCH_RC"
  echo "imu_rows=$IMU_ROWS"
  echo "image_rows=$IMG_ROWS"
  echo "eval_lines=$EVAL_LINES"
  echo "eval_log=$OUTDIR/mipi_cam.eval.log"
} >> "$OUTDIR/result.txt"

if [ "$LAT_RC" -ne 0 ]; then
  echo "RESULT=FAIL reason=latency_script_rc_$LAT_RC" >> "$OUTDIR/result.txt"; exit 4
fi
if [ "$IMU_TOPIC" != "" ] && [ "$IMU_ROWS" -lt 100 ]; then
  echo "RESULT=FAIL reason=insufficient_imu_samples imu=$IMU_ROWS" >> "$OUTDIR/result.txt"; exit 5
fi
if [ "$IMAGE_TOPICS_JSON" != "[]" ] && [ "$IMG_ROWS" -lt 100 ]; then
  echo "RESULT=FAIL reason=insufficient_image_samples image=$IMG_ROWS" >> "$OUTDIR/result.txt"; exit 5
fi
echo "RESULT=OK" >> "$OUTDIR/result.txt"
echo "outdir=$OUTDIR"
grep -E "^(imu_rows|image_rows|RESULT)=" "$OUTDIR/result.txt"
