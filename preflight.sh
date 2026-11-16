#!/usr/bin/env bash
# Demo-day pre-flight (UFLD Part 7): runs every check (PASS/WARN/FAIL), then starts the demo page
# with the chosen config, STOPPED and in DRY RUN. No motor is driven by the checks.
#
#   ./preflight.sh                 checks + start the page (main_demo.py --web --profile config.DEMO_PROFILE)
#   ./preflight.sh --check-only    checks only
#   ./preflight.sh --skip-fps      skip the 10 s FPS test
set -u
cd "$(dirname "$0")" || exit 1
PY=venv/bin/python
START=1
ARGS=()
for a in "$@"; do
  case "$a" in
    --check-only) START=0 ;;
    *) ARGS+=("$a") ;;
  esac
done
if pgrep -f "main_demo.py" >/dev/null || pgrep -f "stream_camera.py" >/dev/null; then
  echo "FAIL  main_demo.py or stream_camera.py is already running (it holds the camera) - stop it first"
  exit 1
fi
$PY preflight_checks.py "${ARGS[@]}"
rc=$?
[ "$START" -eq 1 ] || exit $rc
if [ $rc -ne 0 ]; then
  if [ -t 0 ]; then
    read -r -p "Pre-flight has FAILs. Start the demo page anyway? [y/N] " ans
    [ "$ans" = "y" ] || exit $rc
  else
    exit $rc
  fi
fi
PROFILE=$($PY -c "import config; print(config.DEMO_PROFILE)")
echo "Starting the demo page (profile $PROFILE): STOPPED, DRY RUN on. Ctrl+C to quit."
exec $PY main_demo.py --web --profile "$PROFILE"
