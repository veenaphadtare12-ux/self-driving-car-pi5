#!/usr/bin/env bash
# Part 6: 4 soak runs x MIN minutes, DRY RUN only (page in --source mode is locked to DRY RUN; live: started DRY RUN,
# soak_check asserts dry_run before START). Usage: soak_all.sh MIN
set -u
cd ~/self_driving_car/pi_deploy
MIN=${1:-10}; RUNS=${RUNS:-"ufld_solo:replay ufld_yolo:replay ufld_solo:live ufld_yolo:live"}
SRC=$HOME/car_project/dataset/obstacles/s3
for run in $RUNS; do
  IFS=: read -r prof mode <<< "$run"; out=logs/soak_${prof}_${mode}
  rm -rf "$out"; mkdir -p "$out"
  if [ "$mode" = replay ]; then extra=(--source "$SRC" --no-sensors); else extra=(--dry-run); fi
  echo "=== $(date +%T) $prof $mode: starting server"
  venv/bin/python main_demo.py --web --profile "$prof" "${extra[@]}" > "$out/server.txt" 2>&1 < /dev/null &
  pid=$!
  for i in $(seq 1 90); do
    curl -s -m 2 http://127.0.0.1:8001/status >/dev/null && break; sleep 1
  done
  sleep 8   # models warm up
  venv/bin/python soak_check.py --pid $pid --minutes "$MIN" --out "$out" --server-log "$out/server.txt" > "$out/monitor.txt" 2>&1
  echo "=== $(date +%T) $prof $mode: soak_check exit $?"
  kill -INT $pid; for i in $(seq 1 20); do kill -0 $pid 2>/dev/null || break; sleep 1; done
  kill -0 $pid 2>/dev/null && { echo "server did not stop, SIGKILL"; kill -9 $pid; }
  tail -3 "$out/server.txt"
  sleep 20  # cool down a little between runs
done
echo "=== $(date +%T) ALL DONE"
