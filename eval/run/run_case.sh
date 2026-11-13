#!/usr/bin/env bash
# 板端单用例执行器（eval/00-foundation，review 修正版）
#
# 用法：
#   ./run_case.sh [CASE] [DURATION_S] [OUTDIR]
#   PROJ=/home/sunrise/workspace/hobot_mipi_cam ./run_case.sh C01 30
#   TROS_DISTRO=jazzy ./run_case.sh C01 30          # S600
#
# 读取 eval/cases/<CASE>.yaml 的：
#   eval.duration_s / eval.log_events / recorder.{image,imu,info}_topics
# 命令行 DURATION 优先于 YAML。launch 参数当前仍按 C01（单目）硬编码，
# 后续多用例由 run_suite.py 接管参数映射。
#
# 产出（OUTDIR 下）：
#   mipi_cam.log       被测节点原始 stdout/stderr
#   mipi_cam.eval.log  剥掉 launch 前缀后的纯 [EVAL] 行（分析链用）
#   recorder.log / recorder.csv / recorder.csv.clock / meta.txt / result.txt

set -o pipefail

CASE=${1:-C01}
DURATION_ARG=${2:-}
OUTDIR_ARG=${3:-}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null && pwd)
PROJ=${PROJ:-$(cd "$SCRIPT_DIR/../.." 2>/dev/null && pwd)}
TROS_DISTRO=${TROS_DISTRO:-humble}
TROS_SETUP=${TROS_SETUP:-/opt/tros/${TROS_DISTRO}/setup.bash}

die() {
  echo "RESULT=FAIL reason=$1" >&2
  exit "${2:-1}"
}

[ -n "$PROJ" ] || die proj_not_found 2
CASE_YAML="$PROJ/eval/cases/${CASE}.yaml"
[ -f "$CASE_YAML" ] || die "case_yaml_missing path=$CASE_YAML" 2

[ -f "$TROS_SETUP" ] || die "tros_setup_missing path=$TROS_SETUP" 2

# ---- 读取用例 YAML（PyYAML）----
YAML_TMP=$(mktemp) || die mktemp_failed 2
ARGS_TMP=$(mktemp) || die mktemp_failed 2
if ! python3 - "$CASE_YAML" "$ARGS_TMP" > "$YAML_TMP" <<'PY'
import json
import sys

import yaml

with open(sys.argv[1], encoding='utf-8') as f:
    d = yaml.safe_load(f) or {}
ev = d.get('eval') or {}
rec = d.get('recorder') or {}
duration = ev.get('duration_s', 30)
events = ev.get('log_events', ['all'])
if isinstance(events, list):
    events = ','.join(str(x) for x in events)
print(duration)
print(events)
print(d.get('run_label') or d.get('name') or '')
print(d.get('launch') or '')
print(json.dumps(rec.get('image_topics') or []))
print(json.dumps(rec.get('imu_topics') or []))
print(json.dumps(rec.get('info_topics') or []))
print('True' if ev.get('log_enable', True) else 'False')
load = d.get('load') or {}
print(load.get('cpu_affinity', ''))
print(load.get('cmd', ''))
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
  rm -f "$YAML_TMP" "$ARGS_TMP"
  die case_yaml_parse_failed 2
fi
mapfile -t YV < "$YAML_TMP"
rm -f "$YAML_TMP"
YAML_DURATION=${YV[0]:-30}
YAML_EVENTS=${YV[1]:-all}
RUN_LABEL=${YV[2]:-}
EVAL_LAUNCH=${YV[3]:-}
IMG_TOPICS=${YV[4]:-[]}
IMU_TOPICS=${YV[5]:-[]}
INFO_TOPICS=${YV[6]:-[]}
EVAL_LOG_ENABLE=${YV[7]:-True}
CPU_AFFINITY=${YV[8]:-}
LOAD_CMD=${YV[9]:-}

[ -n "$EVAL_LAUNCH" ] || die launch_not_set 2
[ -f "$PROJ/eval/launch/$EVAL_LAUNCH" ] || die "launch_missing path=$PROJ/eval/launch/$EVAL_LAUNCH" 2

DURATION=${DURATION_ARG:-$YAML_DURATION}
case "$DURATION" in
  ''|*[!0-9]*) die "invalid_duration value=$DURATION" 2 ;;
esac
[ "$DURATION" -gt 0 ] || die "invalid_duration value=$DURATION" 2
EVAL_LOG_EVENTS=${EVAL_LOG_EVENTS:-$YAML_EVENTS}
# 运行目录名必须能看出本轮测什么：<CASE>_<run_label>_<时间戳>
SAFE_LABEL=$(printf '%s' "$RUN_LABEL" | tr -c 'A-Za-z0-9._-' '_' | sed 's/__*/_/g; s/^_//; s/_$//')
if [ -n "$SAFE_LABEL" ]; then
  RUN_PREFIX="${CASE}_${SAFE_LABEL}"
else
  RUN_PREFIX="$CASE"
fi
OUTDIR=${OUTDIR_ARG:-$PROJ/eval/logs/${RUN_PREFIX}_$(date +%Y%m%d_%H%M%S)}
START_EPOCH=$(date +%s)

# OUTDIR 必须是新目录：防止旧 CSV/旧 EVAL 让本轮假通过
if [ -e "$OUTDIR" ] && [ -n "$(find "$OUTDIR" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]; then
  die "outdir_not_empty path=$OUTDIR" 2
fi
mkdir -p "$OUTDIR" || die "outdir_create_failed path=$OUTDIR" 2
mv "$ARGS_TMP" "$OUTDIR/launch.args" || die launch_args_write_failed 2
LAUNCH_ARGS_FILE="$OUTDIR/launch.args"

# ---- TROS + 本仓库交叉编译产物 ----
# shellcheck disable=SC1090
source "$TROS_SETUP" >/dev/null 2>&1 || die "tros_setup_source_failed path=$TROS_SETUP" 2
[ -f "$PROJ/install/setup.bash" ] || die "missing_install_setup path=$PROJ/install/setup.bash" 2
SETUP_ERR=$(mktemp) || die mktemp_failed 2
# shellcheck disable=SC1091
source "$PROJ/install/setup.bash" 2>"$SETUP_ERR"
# 交叉编译产物会链到容器内 /mnt/cc_ws/tros_ws/install；板上不存在属预期。
grep -v '^not found: "/mnt/cc_ws/' "$SETUP_ERR" >&2 || true
rm -f "$SETUP_ERR"

PKG_PREFIX=$(ros2 pkg prefix mipi_cam 2>/dev/null || true)
EVAL_PREFIX=$(ros2 pkg prefix mipi_cam_eval 2>/dev/null || true)
[ -n "$PKG_PREFIX" ] || die package_mipi_cam_not_found 3
[ -n "$EVAL_PREFIX" ] || die package_mipi_cam_eval_not_found 3
export PKG_PREFIX
export EVAL_LOG_ENABLE
export CPU_AFFINITY
export LOAD_CMD

# YAML params 必须全部被 launch 声明，防止参数被静默丢弃（例如 io_method）
SHOW_ARGS=$(ros2 launch mipi_cam_eval "$EVAL_LAUNCH" --show-args 2>/dev/null || true)
if [ -z "$SHOW_ARGS" ]; then
  echo "WARN launch_show_args_failed, skip param coverage check" >&2
else
  UNSUPPORTED=""
  while IFS= read -r line; do
    key=${line%%:=*}
    [ -n "$key" ] || continue
    if ! printf '%s\n' "$SHOW_ARGS" | grep -q "^    '${key}':"; then
      UNSUPPORTED="$UNSUPPORTED $key"
    fi
  done < "$LAUNCH_ARGS_FILE"
  if [ -n "$UNSUPPORTED" ]; then
    die "launch_args_unsupported:$UNSUPPORTED" 2
  fi
fi

{
  echo "case=$CASE"
  echo "run_label=$RUN_LABEL"
  echo "case_yaml=$CASE_YAML"
  echo "launch=$EVAL_LAUNCH"
  echo "launch_args=$(tr '\n' ' ' < "$LAUNCH_ARGS_FILE" 2>/dev/null)"
  echo "duration_s=$DURATION"
  echo "outdir=$OUTDIR"
  echo "run_start=$(date -Iseconds)"
  echo "start_epoch=$START_EPOCH"
  echo "kernel=$(uname -r)"
  echo "tros_setup=$TROS_SETUP"
  echo "mipi_cam_prefix=$PKG_PREFIX"
  echo "mipi_cam_eval_prefix=$EVAL_PREFIX"
  echo "eval_log_events=$EVAL_LOG_EVENTS"
  echo "eval_log_enable=$EVAL_LOG_ENABLE"
  echo "cpu_affinity=$CPU_AFFINITY"
  echo "load_cmd=$LOAD_CMD"
  echo "image_topics=$IMG_TOPICS"
  echo "info_topics=$INFO_TOPICS"
  echo "board_loadavg=$(cut -d' ' -f1-3 /proc/loadavg 2>/dev/null)"
  echo "uptime_s=$(cut -d' ' -f1 /proc/uptime 2>/dev/null)"
  echo "git_commit=$(git -C "$PROJ" rev-parse --short HEAD 2>/dev/null || echo unknown)"
} > "$OUTDIR/meta.txt"

REC_PID=""
LAUNCH_PID=""
LOAD_PID=""
REC_PGID=""
LAUNCH_PGID=""
LOAD_PGID=""

cleanup() {
  local pg
  for pg in "$LOAD_PGID" "$LAUNCH_PGID" "$REC_PGID"; do
    [ -n "$pg" ] || continue
    kill -INT -"$pg" 2>/dev/null || true
  done
  for pg in "$LOAD_PID" "$LAUNCH_PID" "$REC_PID"; do
    [ -n "$pg" ] || continue
    kill -INT "$pg" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# ---- 1) recorder：先起，等就绪后再起被测节点 ----
export REC_CSV="$OUTDIR/recorder.csv"
export REC_PIDFILE="$OUTDIR/recorder.pgid"
export REC_DUR=$((DURATION + 20))
export IMG_TOPICS IMU_TOPICS INFO_TOPICS
export EVAL_LOG_EVENTS

setsid -w bash -c '
  echo $$ > "$REC_PIDFILE"
  exec timeout --signal=INT --kill-after=10 "$REC_DUR" \
    ros2 run mipi_cam_eval recorder --ros-args \
    -p csv_path:="$REC_CSV" \
    -p image_topics:="$IMG_TOPICS" \
    -p imu_topics:="$IMU_TOPICS" \
    -p info_topics:="$INFO_TOPICS"
' > "$OUTDIR/recorder.log" 2>&1 &
REC_PID=$!

REC_READY=0
for _ in $(seq 1 100); do
  if [ -s "$OUTDIR/recorder.csv.clock" ] && grep -q "recorder started" "$OUTDIR/recorder.log" 2>/dev/null; then
    REC_READY=1
    break
  fi
  kill -0 "$REC_PID" 2>/dev/null || break
  sleep 0.1
done
if [ "$REC_READY" -ne 1 ]; then
  echo "RESULT=FAIL reason=recorder_start_failed" | tee "$OUTDIR/result.txt" >&2
  exit 5
fi
[ -s "$OUTDIR/recorder.pgid" ] && REC_PGID=$(cat "$OUTDIR/recorder.pgid")

# ---- 1b) 可选负载（正向对照）：YAML load.cmd，跑在独立进程组 ----
if [ -n "$LOAD_CMD" ]; then
  export LOAD_PIDFILE="$OUTDIR/load.pgid"
  setsid -w bash -c 'echo $$ > "$LOAD_PIDFILE"; exec bash -c "$LOAD_CMD"' \
    > "$OUTDIR/load.log" 2>&1 &
  LOAD_PID=$!
  for _ in $(seq 1 50); do
    [ -s "$OUTDIR/load.pgid" ] && break
    sleep 0.05
  done
  [ -s "$OUTDIR/load.pgid" ] && LOAD_PGID=$(cat "$OUTDIR/load.pgid")
fi

# ---- 2) 被测节点：时长到后 SIGINT，launch 优雅收栈 ----
export LAUNCH_PIDFILE="$OUTDIR/launch.pgid"
export LAUNCH_DUR="$DURATION"
export EVAL_LAUNCH LAUNCH_ARGS_FILE
setsid -w bash -c '
  echo $$ > "$LAUNCH_PIDFILE"
  mapfile -t args < "$LAUNCH_ARGS_FILE"
  wrap=""
  if [ -n "$CPU_AFFINITY" ]; then wrap="taskset -c $CPU_AFFINITY"; fi
  exec timeout --signal=INT --kill-after=15 "$LAUNCH_DUR" \
    $wrap ros2 launch mipi_cam_eval "$EVAL_LAUNCH" "${args[@]}" \
    eval_log_enable:="$EVAL_LOG_ENABLE" \
    eval_log_events:="$EVAL_LOG_EVENTS" \
    log_level:=warn
' > "$OUTDIR/mipi_cam.log" 2>&1 &
LAUNCH_PID=$!
# 等内层 bash 写入 PGID（setsid 新会话的首进程）
for _ in $(seq 1 50); do
  [ -s "$OUTDIR/launch.pgid" ] && break
  sleep 0.05
done
[ -s "$OUTDIR/launch.pgid" ] && LAUNCH_PGID=$(cat "$OUTDIR/launch.pgid")

wait "$LAUNCH_PID"
LAUNCH_RC=$?
LAUNCH_PID=""
LAUNCH_PGID=""

# ---- 2b) 收尾负载 ----
if [ -n "$LOAD_PGID" ]; then
  kill -INT -"$LOAD_PGID" 2>/dev/null || true
fi
if [ -n "$LOAD_PID" ]; then
  wait "$LOAD_PID" 2>/dev/null || true
  LOAD_PID=""
  LOAD_PGID=""
fi

# ---- 3) 收尾 recorder ----
sleep 1
if [ -n "$REC_PGID" ]; then
  kill -INT -"$REC_PGID" 2>/dev/null || true
fi
if [ -n "$REC_PID" ]; then
  wait "$REC_PID" 2>/dev/null || true
  REC_PID=""
  REC_PGID=""
fi

# ---- 4) 统计与判据 ----
CSV_ROWS=0
CSV_MIN=0
CSV_MAX=0
TOPIC_COUNTS=""
if [ -f "$OUTDIR/recorder.csv" ]; then
  read -r CSV_MIN CSV_MAX CSV_ROWS < <(awk -F, '
    NR > 1 {
      v = $4 + 0
      if (min == "" || v < min) min = v
      if (max == "" || v > max) max = v
      c[$1]++
      n++
    }
    END {
      if (n > 0) printf "%.0f %.0f %d\n", min, max, n
      else printf "0 0 0\n"
    }' "$OUTDIR/recorder.csv" 2>/dev/null || echo "0 0 0")
  TOPIC_COUNTS=$(awk -F, 'NR > 1 { c[$1]++ } END { for (k in c) printf "%s=%d ", k, c[k] }' \
    "$OUTDIR/recorder.csv" 2>/dev/null || true)
fi
COVER_NS=$((CSV_MAX - CSV_MIN))
[ "$COVER_NS" -lt 0 ] && COVER_NS=0
MIN_COVER_NS=$((DURATION * 1000000000 / 2))

CLOCK_OK=0
[ -s "$OUTDIR/recorder.csv.clock" ] && CLOCK_OK=1

EVAL_LOG="$OUTDIR/mipi_cam.eval.log"
# 只剥 launch 的 "[mipi_cam-<n>] " 前缀；契约行必须带 ts_mono/event
grep -E '\[EVAL\] ts_mono=[0-9]+ event=' "$OUTDIR/mipi_cam.log" 2>/dev/null \
  | sed -E 's/^\[mipi_cam-[0-9]+\] //' > "$EVAL_LOG" || true
EVAL_LINES=$(grep -cE '^\[EVAL\] ts_mono=[0-9]+ event=' "$EVAL_LOG" 2>/dev/null || true)
EVAL_LINES=${EVAL_LINES:-0}

# 必需 topic（YAML recorder.image_topics）必须 >0；IMU/info 只告警不判失败
MISSING=""
if [ "$IMG_TOPICS" != "[]" ]; then
  while IFS= read -r t; do
    [ -n "$t" ] || continue
    cnt=$(awk -F, -v t="$t" 'NR > 1 && $1 == t { c++ } END { print c + 0 }' \
      "$OUTDIR/recorder.csv" 2>/dev/null || echo 0)
    [ "$cnt" -gt 0 ] || MISSING="$MISSING $t"
  done < <(python3 -c 'import json,sys; [print(x) for x in json.loads(sys.argv[1])]' "$IMG_TOPICS")
fi

WARNINGS=""
for pair in "imu:$IMU_TOPICS" "info:$INFO_TOPICS"; do
  label=${pair%%:*}
  topics=${pair#*:}
  [ "$topics" = "[]" ] && continue
  while IFS= read -r t; do
    [ -n "$t" ] || continue
    cnt=$(awk -F, -v t="$t" 'NR > 1 && $1 == t { c++ } END { print c + 0 }' \
      "$OUTDIR/recorder.csv" 2>/dev/null || echo 0)
    [ "$cnt" -gt 0 ] || WARNINGS="$WARNINGS ${label}_topic_empty:$t"
  done < <(python3 -c 'import json,sys; [print(x) for x in json.loads(sys.argv[1])]' "$topics")
done

END_EPOCH=$(date +%s)
ELAPSED=$((END_EPOCH - START_EPOCH))
CLOCK_JUMP=0
if [ "$ELAPSED" -lt 0 ] || [ "$ELAPSED" -gt $((DURATION + 60)) ]; then
  CLOCK_JUMP=1
fi

RC_OK=0
case "$LAUNCH_RC" in 0|124|130|137|143) RC_OK=1 ;; esac
RESULT=FAIL
FAIL_REASON=""
if [ "$RC_OK" -ne 1 ]; then
  FAIL_REASON="$FAIL_REASON launch_rc=$LAUNCH_RC"
fi
[ "$CSV_ROWS" -ge 1 ] || FAIL_REASON="$FAIL_REASON csv_empty"
[ "$EVAL_LINES" -ge 1 ] || FAIL_REASON="$FAIL_REASON eval_empty"
[ "$CLOCK_OK" -eq 1 ] || FAIL_REASON="$FAIL_REASON clock_missing"
[ "$COVER_NS" -ge "$MIN_COVER_NS" ] || FAIL_REASON="$FAIL_REASON coverage_short"
[ -z "$MISSING" ] || FAIL_REASON="$FAIL_REASON missing_topics=$MISSING"
if [ -z "$FAIL_REASON" ]; then
  RESULT=OK
else
  RESULT="FAIL reason=${FAIL_REASON# }"
fi

{
  echo "case=$CASE"
  echo "launch=$EVAL_LAUNCH"
  echo "cpu_affinity=$CPU_AFFINITY"
  echo "load_cmd=$LOAD_CMD"
  echo "duration_s=$DURATION"
  echo "outdir=$OUTDIR"
  echo "run_end=$(date -Iseconds)"
  echo "elapsed_s=$ELAPSED"
  echo "launch_rc=$LAUNCH_RC"
  echo "csv_rows=$CSV_ROWS"
  echo "csv_cover_s=$(awk -v ns="$COVER_NS" 'BEGIN{printf "%.2f", ns/1e9}')"
  echo "eval_lines=$EVAL_LINES"
  echo "clock_ok=$CLOCK_OK"
  echo "clock_jump_suspected=$CLOCK_JUMP"
  echo "topic_counts=$TOPIC_COUNTS"
  echo "missing_required_topics=$MISSING"
  echo "warnings=$WARNINGS"
  echo "eval_log=$EVAL_LOG"
  echo "RESULT=$RESULT"
} | tee "$OUTDIR/result.txt"

[ "$RESULT" = "OK" ]
