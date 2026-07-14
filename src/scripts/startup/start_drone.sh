#!/bin/bash

# Drone Initialization Script (tmux version)
# Launches MAVProxy, MAVROS, and the camera in split terminal panes

SESSION_NAME="drone_init"

# Camera profile: selects src/config/camera_<profile>.yaml for usb_cam.
#   gopro_hero4 (default) = GoPro HERO4 via HDMI-USB capture card
#   c270                  = legacy Logitech C270 webcam
# Override per run:  CAMERA_PROFILE=c270 ./start_drone.sh
CAMERA_PROFILE="${CAMERA_PROFILE:-gopro_hero4}"

echo "=========================================="
echo "Drone Initialization Script (tmux)"
echo "=========================================="
echo ""
echo "This script will launch all services in a single tmux session"
echo "with split panes for easy monitoring"
echo ""
echo "Camera profile: $CAMERA_PROFILE"
echo ""

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "ERROR: tmux is not installed"
    echo "Install with: sudo apt install tmux"
    exit 1
fi

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

echo "Creating tmux session: $SESSION_NAME"
echo ""
echo "Navigation tips:"
echo "  - Ctrl+b then arrow keys: Switch between panes"
echo "  - Ctrl+b then d: Detach from session (services keep running)"
echo "  - tmux attach -t $SESSION_NAME: Re-attach to session"
echo "  - Ctrl+c in any pane: Stop that service"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Create new tmux session with first command (MAVProxy)
tmux new-session -d -s $SESSION_NAME -n "Drone Services"

# Set up the layout and run commands
tmux send-keys -t $SESSION_NAME "echo '=== MAVProxy ==='" C-m
tmux send-keys -t $SESSION_NAME "sleep 1" C-m
tmux send-keys -t $SESSION_NAME "mavproxy.py --master=/dev/serial0 --baudrate=57600" C-m

# Split horizontally and run MAVROS (wait 5 seconds)
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== MAVROS (waiting 5s) ==='" C-m
tmux send-keys -t $SESSION_NAME "sleep 5" C-m
tmux send-keys -t $SESSION_NAME "ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/serial0:57600" C-m

# Split the right pane vertically and run Camera (wait 10 seconds total)
# All camera settings (device, resolution, pixel format, calibration file) come
# from the profile yaml, resolved from the installed package share — no more
# hardcoded clone paths. \$(...) is escaped so it evaluates inside the pane.
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== Camera: ${CAMERA_PROFILE} (waiting 10s) ==='" C-m
tmux send-keys -t $SESSION_NAME "sleep 10" C-m
tmux send-keys -t $SESSION_NAME "ros2 run usb_cam usb_cam_node_exe --ros-args --params-file \$(ros2 pkg prefix warehouse_drone_nav)/share/warehouse_drone_nav/config/camera_${CAMERA_PROFILE}.yaml -r __ns:=/camera" C-m

# Attach to the session
echo "Attaching to tmux session..."
tmux attach-session -t $SESSION_NAME