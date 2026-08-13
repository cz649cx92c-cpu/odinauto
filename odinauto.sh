#!/usr/bin/env bash
set -eo pipefail

BASE=/home/orangepi/ugv/odinauto
RUN_DIR="$BASE/run"
GOAL_UI="$BASE/goal_ui/run.sh"
ODIN_CONFIG=/home/orangepi/ugv/ros2_ws/src/odin_ros_driver/config/control_command.yaml
ODIN_CALIB=/home/orangepi/ugv/ros2_ws/src/odin_ros_driver/config/calib.yaml
LIDAR_BIN=/home/orangepi/ugv/install/lidar_pkg/lib/lidar_pkg/lidar_node
LIDAR_CONFIG=/home/orangepi/ugv/install/lidar_pkg/share/lidar_pkg/config/lidar_params.yaml
LIDAR_DEVICE=/dev/serial/by-id/usb-STC_STC_USB_Serial-if00

mkdir -p "$RUN_DIR"
exec 9>"$RUN_DIR/odinauto.lock"
if ! flock -n 9; then
    echo 'Another odinauto start/stop operation is running.' >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source /home/orangepi/ugv/install/setup.bash
source /home/orangepi/ugv/ros2_ws/install/setup.bash
source "$BASE/SRU-Odin/Deployment_ros2/install/setup.bash"

ROS_SETUP="source /opt/ros/humble/setup.bash; source /home/orangepi/ugv/install/setup.bash; source /home/orangepi/ugv/ros2_ws/install/setup.bash; source $BASE/SRU-Odin/Deployment_ros2/install/setup.bash; export AMENT_PREFIX_PATH=$BASE/SRU-Odin/Deployment_ros2/install/sru_nav_odin_ros2:\$AMENT_PREFIX_PATH; export PYTHONPATH=$BASE/SRU-Odin/Deployment_ros2/install/sru_nav_odin_ros2/lib/python3.10/site-packages:$BASE/SRU-Odin/Deployment_ros2/build/sru_nav_odin_ros2:\$PYTHONPATH"

is_running() {
    local pid_file="$RUN_DIR/$1.pid"
    local pid state
    [[ -f "$pid_file" ]] || return 1
    pid=$(cat "$pid_file")
    kill -0 "$pid" 2>/dev/null || return 1
    state=$(ps -o stat= -p "$pid" 2>/dev/null) || return 1
    [[ -n "$state" && "${state:0:1}" != 'Z' ]]
}

start_component() {
    local name=$1
    shift
    if is_running "$name"; then
        echo "[running] $name (pid $(cat "$RUN_DIR/$name.pid"))"
        return
    fi
    rm -f "$RUN_DIR/$name.pid"
    setsid bash -lc "$ROS_SETUP; exec $*" \
        >"$RUN_DIR/$name.log" 2>&1 < /dev/null 9>&- &
    local pid=$!
    echo "$pid" >"$RUN_DIR/$name.pid"
    echo "[started] $name (pid $pid)"
}

scan_publisher_count() {
    timeout 3 ros2 topic info /scan 2>/dev/null |
        awk '/^Publisher count:/ {print $3; exit}'
}

scan_publisher_exists() {
    [[ "$(scan_publisher_count)" =~ ^[1-9][0-9]*$ ]]
}

scan_ready() {
    [[ "$(scan_publisher_count)" == '1' ]] &&
        timeout 3 ros2 topic echo --once /scan \
            >/dev/null 2>&1
}

managed_lidar_running() {
    local pid command
    [[ -f "$RUN_DIR/lidar.owned" ]] || return 1
    is_running lidar || return 1
    pid=$(cat "$RUN_DIR/lidar.pid")
    command=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null) || return 1
    [[ "$command" == *"$LIDAR_BIN"* ]]
}

start_lidar() {
    if managed_lidar_running; then
        echo "[running] lidar (pid $(cat "$RUN_DIR/lidar.pid"), managed)"
        return 0
    fi
    rm -f "$RUN_DIR/lidar.pid" "$RUN_DIR/lidar.owned"
    if scan_publisher_exists; then
        echo '[running] lidar (external /scan publisher reused)'
        return 0
    fi
    [[ -x "$LIDAR_BIN" ]] || {
        echo "Lidar driver does not exist: $LIDAR_BIN" >&2
        return 1
    }
    [[ -e "$LIDAR_DEVICE" ]] || {
        echo "Lidar device does not exist: $LIDAR_DEVICE" >&2
        return 1
    }
    setsid bash -lc "$ROS_SETUP; exec $LIDAR_BIN --ros-args --params-file $LIDAR_CONFIG -p port_name:=$LIDAR_DEVICE" \
        >"$RUN_DIR/lidar.log" 2>&1 < /dev/null 9>&- &
    local pid=$!
    echo "$pid" >"$RUN_DIR/lidar.pid"
    echo "$pid" >"$RUN_DIR/lidar.owned"
    echo "[started] lidar (pid $pid, managed)"
}

stop_lidar() {
    if [[ ! -f "$RUN_DIR/lidar.owned" ]]; then
        rm -f "$RUN_DIR/lidar.pid"
        echo '[retained] lidar (external ownership)'
        return 0
    fi
    if managed_lidar_running; then
        stop_component lidar
    else
        rm -f "$RUN_DIR/lidar.pid"
        echo '[stopped] lidar (managed process already exited)'
    fi
    rm -f "$RUN_DIR/lidar.owned"
}

stop_unmanaged_can_nodes() {
    local managed_pid='' pid
    if is_running can; then
        managed_pid=$(cat "$RUN_DIR/can.pid")
    fi
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        if [[ -n "$managed_pid" ]] && ps -o pid= --ppid "$managed_pid" | grep -qw "$pid"; then
            continue
        fi
        kill -TERM "$pid" 2>/dev/null || true
    done < <(pgrep -f '/yhs_can_control/lib/yhs_can_control/yhs_can_control_node' || true)
}

stop_component() {
    local name=$1
    local pid_file="$RUN_DIR/$name.pid"
    if ! is_running "$name"; then
        rm -f "$pid_file"
        echo "[stopped] $name"
        return
    fi
    local pid
    pid=$(cat "$pid_file")
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 15); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi
    rm -f "$pid_file"
    echo "[stopped] $name"
}

stop_orphan_processes() {
    local patterns=(
        '/home/orangepi/ugv/ros2_ws/install/lib/odin_ros_driver/host_sdk_sample'
        '/home/orangepi/ugv/ros2_ws/install/lib/odin_ros_driver/pcd2depth_ros2_node'
        '/sru_nav_odin_ros2/lib/sru_nav_odin_ros2/'
        '/home/orangepi/ugv/install/yhs_can_control/lib/yhs_can_control/yhs_can_control_node'
        '/home/orangepi/ugv/odinauto/goal_ui/server.py'
    )
    local pattern
    for pattern in "${patterns[@]}"; do
        pkill -TERM -f "$pattern" 2>/dev/null || true
    done
    sleep 0.5
    for pattern in "${patterns[@]}"; do
        pkill -KILL -f "$pattern" 2>/dev/null || true
    done
}

can_interface_ready() {
    local link_info
    link_info=$(ip -details link show can0 2>/dev/null) || return 1
    grep -Eq '<[^>]*UP[^>]*LOWER_UP[^>]*>' <<<"$link_info" || return 1
    grep -Eq 'bitrate 500000([[:space:]]|$)' <<<"$link_info"
}

wait_until_ready() {
    local deadline=$((SECONDS + 25))
    local nodes=''
    while (( SECONDS < deadline )); do
        nodes=$(timeout 3 ros2 node list 2>/dev/null || true)
        local deployment_nodes
        deployment_nodes=$(pgrep -af "$BASE/SRU-Odin/Deployment_ros2/install/sru_nav_odin_ros2/lib/sru_nav_odin_ros2/" || true)
        if is_running can &&
           is_running odin &&
           is_running depth &&
           is_running navigation &&
           is_running goal_ui &&
           scan_ready &&
           can_interface_ready &&
           grep -qx '/lidar_node' <<<"$nodes" &&
           grep -qx '/lydros_node' <<<"$nodes" &&
           grep -qx '/depth_image_ros2_node' <<<"$nodes" &&
           grep -qx '/sru_nav_node' <<<"$nodes" &&
           grep -qx '/sru_local_planner' <<<"$nodes" &&
           grep -qx '/sru_pointcloud_safety' <<<"$nodes" &&
           grep -qx '/sru_twist_to_yhs' <<<"$nodes" &&
           grep -qx '/yhs_can_control_node' <<<"$nodes" &&
           grep -q '/sru_nav_node' <<<"$deployment_nodes" &&
           grep -q '/local_planner' <<<"$deployment_nodes" &&
           grep -q '/pointcloud_safety' <<<"$deployment_nodes" &&
           grep -q '/twist_to_yhs' <<<"$deployment_nodes" &&
           curl -fsS http://127.0.0.1:8090/ >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.5
    done
    echo 'Startup did not become ready within 25 seconds.' >&2
    echo 'Logs are under /home/orangepi/ugv/odinauto/run/' >&2
    return 1
}

do_start() {
    # Components are independent at process startup. ROS subscriptions connect
    # automatically as publishers appear, so parallel launch avoids fixed waits.
    stop_unmanaged_can_nodes
    start_component can ros2 launch yhs_can_control yhs_can_control.launch.py
    start_component odin ros2 run odin_ros_driver host_sdk_sample --ros-args \
        -p config_file:="$ODIN_CONFIG"
    start_component depth ros2 run odin_ros_driver pcd2depth_ros2_node --ros-args \
        -p calib_file_path:="$ODIN_CALIB"
    start_lidar || return 1
    start_component navigation ros2 launch sru_nav_odin_ros2 sru_nav_odin.launch.py
    start_component goal_ui "$GOAL_UI"

    wait_until_ready || return 1
    if ! timeout 6 ros2 topic pub --once /sru/cancel_navigation \
            std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1; then
        echo 'Failed to initialize navigation stop state.' >&2
        return 1
    fi
    if ! timeout 6 ros2 topic pub --once /io_cmd yhs_can_interfaces/msg/IoCmd \
            '{io_cmd_unlock: true}' >/dev/null 2>&1; then
        echo 'Failed to unlock chassis.' >&2
        return 1
    fi
    echo 'READY http://192.168.101.212:8090/'
}

do_stop() {
    # Stop intent reaches both navigation and the chassis before processes exit.
    { timeout 3 ros2 topic pub --once /sru/cancel_navigation \
        std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; } &
    local cancel_pid=$!
    { timeout 3 ros2 topic pub --once /ctrl_cmd yhs_can_interfaces/msg/CtrlCmd \
        '{ctrl_cmd_gear: 6, ctrl_cmd_x_linear: 0.0, ctrl_cmd_y_linear: 0.0, ctrl_cmd_z_angular: 0.0}' \
        >/dev/null 2>&1 || true; } &
    local zero_pid=$!
    wait "$cancel_pid" "$zero_pid" || true
    sleep 0.2
    stop_component navigation
    stop_lidar
    stop_component depth
    stop_component odin
    stop_component goal_ui
    stop_component can
    stop_orphan_processes
    echo 'STOPPED'
}

do_status() {
    for name in can odin depth navigation goal_ui; do
        if is_running "$name"; then
            echo "RUNNING $name pid=$(cat "$RUN_DIR/$name.pid")"
        else
            echo "STOPPED $name"
        fi
    done
    if managed_lidar_running; then
        echo "RUNNING lidar pid=$(cat "$RUN_DIR/lidar.pid") managed"
    elif scan_publisher_exists; then
        echo 'RUNNING lidar external'
    else
        echo 'STOPPED lidar'
    fi
    echo 'ROS nodes:'
    timeout 4 ros2 node list 2>/dev/null | sort -u || true
    if curl -fsS http://127.0.0.1:8090/ >/dev/null 2>&1; then
        echo 'WEB http://192.168.101.212:8090/ OK'
    else
        echo 'WEB stopped'
    fi
}

if [[ $# -eq 0 ]]; then
    case "$(basename "$0")" in
        start_all.sh) set -- start ;;
        stop_all.sh) set -- stop ;;
    esac
fi

case "${1:-}" in
    start) do_start ;;
    stop) do_stop ;;
    restart) do_stop; do_start ;;
    status) do_status ;;
    *) echo "Usage: $0 {start|stop|restart|status}" >&2; exit 2 ;;
esac
