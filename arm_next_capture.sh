#!/usr/bin/env bash
set -eo pipefail

BASE=/home/orangepi/ugv/odinauto
RUN_DIR="$BASE/run"
CAPTURE_SCRIPT="$BASE/capture_next_navigation.py"

mkdir -p "$RUN_DIR" "$BASE/captures"
if [[ -f "$RUN_DIR/capture_next.pid" ]]; then
    pid=$(cat "$RUN_DIR/capture_next.pid")
    if kill -0 "$pid" 2>/dev/null; then
        echo "ARMED capture_next pid=$pid"
        exit 0
    fi
    rm -f "$RUN_DIR/capture_next.pid"
fi

source /opt/ros/humble/setup.bash
source /home/orangepi/ugv/install/setup.bash
source /home/orangepi/ugv/ros2_ws/install/setup.bash
source "$BASE/SRU-Odin/Deployment_ros2/install/setup.bash"

setsid python3 "$CAPTURE_SCRIPT" \
    >"$RUN_DIR/capture_next.log" 2>&1 < /dev/null &
pid=$!
echo "$pid" >"$RUN_DIR/capture_next.pid"
sleep 1
if ! kill -0 "$pid" 2>/dev/null; then
    echo 'Failed to arm next-navigation capture.' >&2
    tail -n 20 "$RUN_DIR/capture_next.log" >&2 || true
    exit 1
fi
echo "ARMED capture_next pid=$pid"
