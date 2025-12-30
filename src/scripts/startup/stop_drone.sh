#!/bin/bash

# Stop all drone services

echo "=========================================="
echo "Stopping Drone Services"
echo "=========================================="
echo ""

# Function to check if process is running
check_process() {
    pgrep -f "$1" > /dev/null
    return $?
}

# Stop MAVProxy
echo "[1/3] Stopping MAVProxy..."
if check_process "mavproxy.py"; then
    pkill -f mavproxy.py
    sleep 1
    if check_process "mavproxy.py"; then
        pkill -9 -f mavproxy.py
    fi
    echo "  ✓ MAVProxy stopped"
else
    echo "  ℹ MAVProxy not running"
fi

# Stop MAVROS
echo "[2/3] Stopping MAVROS..."
if check_process "mavros_node"; then
    pkill -f mavros_node
    sleep 1
    if check_process "mavros_node"; then
        pkill -9 -f mavros_node
    fi
    echo "  ✓ MAVROS stopped"
else
    echo "  ℹ MAVROS not running"
fi

# Stop USB Camera
echo "[3/3] Stopping USB Camera..."
if check_process "usb_cam_node_exe"; then
    pkill -f usb_cam_node_exe
    sleep 1
    if check_process "usb_cam_node_exe"; then
        pkill -9 -f usb_cam_node_exe
    fi
    echo "  ✓ USB Camera stopped"
else
    echo "  ℹ USB Camera not running"
fi

# Kill any tmux sessions
echo ""
echo "Checking for tmux sessions..."
if tmux has-session -t drone_init 2>/dev/null; then
    tmux kill-session -t drone_init
    echo "  ✓ Tmux session 'drone_init' killed"
else
    echo "  ℹ No tmux session found"
fi

echo ""
echo "=========================================="
echo "All drone services stopped"
echo "=========================================="