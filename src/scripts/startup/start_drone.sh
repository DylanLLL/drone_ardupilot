#!/bin/bash

# Drone Initialization Script (tmux version)
# Launches MAVProxy, MAVROS, and USB Camera in split terminal panes

SESSION_NAME="drone_init"

echo "=========================================="
echo "Drone Initialization Script (tmux)"
echo "=========================================="
echo ""
echo "This script will launch all services in a single tmux session"
echo "with split panes for easy monitoring"
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
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== USB Camera (waiting 10s) ==='" C-m
tmux send-keys -t $SESSION_NAME "sleep 10" C-m
tmux send-keys -t $SESSION_NAME "ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480 -p pixel_format:=yuyv -p camera_frame_id:=camera_frame -p camera_info_url:='file:///home/drone/ros2_ws/src/drone_ardupilot/src/config/camera_calibration.yaml' -r __ns:=/camera" C-m

# Attach to the session
echo "Attaching to tmux session..."
tmux attach-session -t $SESSION_NAME