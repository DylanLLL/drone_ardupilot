# Quick Start Guide

## Initial Setup (One-time)

### 1. Install ROS2 Humble on Raspberry Pi
```bash
# Install ROS2 Humble
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2. Install Dependencies
```bash
# Install ROS2 packages
sudo apt install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-tf2-ros

# Install GeographicLib for MAVROS
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install Python packages
pip3 install -r requirements.txt
```

### 3. Setup Workspace
```bash
# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy warehouse_drone_nav package to src/
# (You're already here if reading this!)

# Build
cd ~/ros2_ws
colcon build --packages-select warehouse_drone_nav
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 4. Configure ArduPilot
Use Mission Planner to configure:
```
EK3_SRC1_POSXY = 6 (ExternalNav)
EK3_SRC1_POSZ = 6 (ExternalNav)
VISO_TYPE = 1 (MAV)
```

## Daily Workflow

### Step 1: Power On
1. Connect Pixhawk to RPi via USB
2. Turn on RC transmitter
3. Connect LiPo battery
4. Wait for Pixhawk to boot (LED patterns indicate status)

### Step 2: Start MAVROS (Terminal 1)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
```
Wait for "CON: Got HEARTBEAT" message

### Step 3: Start Camera (Terminal 2)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=/dev/video0 \
    -p image_width:=640 \
    -p image_height:=480 \
    -p camera_frame_id:=camera_frame \
    -r __ns:=/camera
```

### Step 4: Test ArUco Detection (Terminal 3)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch warehouse_drone_nav aruco_test.launch.py
```

### Step 5: View Detection (Terminal 4 - Optional)
```bash
ros2 run rqt_image_view rqt_image_view /aruco/detection_image
```
Verify markers are detected properly

### Step 6: Launch Navigation System (Terminal 3)
Stop the test launch (Ctrl+C) and start full system:
```bash
ros2 launch warehouse_drone_nav warehouse_nav.launch.py
```

### Step 7: Arm and Test in STABILIZE
1. Switch RC to STABILIZE mode
2. Arm using RC throttle + yaw (or via Mission Planner)
3. Test manual control
4. Land and disarm

### Step 8: Switch to GUIDED Mode and Run Mission
1. Switch RC to GUIDED mode
2. Start mission:
```bash
# Terminal 5
ros2 topic pub --once /mission/command std_msgs/msg/String "data: 'start'"
```

### Step 9: Monitor (Terminal 6)
```bash
# Watch mission status
ros2 topic echo /mission/status

# Or watch position
ros2 topic echo /drone/local_position
```

### Emergency Procedures
- **Switch to STABILIZE**: Take manual control anytime
- **Emergency Stop**: 
  ```bash
  ros2 topic pub --once /mission/command std_msgs/msg/String "data: 'land'"
  ```
- **RC Override**: Always have RC transmitter ready

## Testing Checklist

Before autonomous flight:
- [ ] All ROS2 nodes running without errors
- [ ] MAVROS connected (green in Mission Planner)
- [ ] Camera feed visible
- [ ] ArUco markers detected
- [ ] Position estimate being published
- [ ] RC transmitter on and in hand
- [ ] Flight area clear
- [ ] Battery > 50%

## Common Issues

**"MAVROS not connecting"**
```bash
# Check device
ls /dev/ttyUSB*
# Try different baud rates: 57600, 115200, 921600
```

**"Camera not found"**
```bash
# List cameras
ls /dev/video*
# Try v4l2-ctl --list-devices
```

**"Markers not detected"**
- Check lighting (no glare/shadows)
- Verify marker size in config
- Ensure camera focus is correct
- Try different camera height

**"Position jumping/unstable"**
- Recalibrate camera
- Check marker map coordinates
- Add more markers
- Verify marker placement

## Useful Commands

```bash
# List all topics
ros2 topic list

# List all nodes
ros2 node list

# Check node info
ros2 node info /aruco_detector

# Monitor a topic
ros2 topic echo /topic_name

# Get topic frequency
ros2 topic hz /topic_name

# View transforms
ros2 run tf2_ros tf2_echo map base_link
```

## Next Steps

1. Test with a single marker first
2. Add more markers gradually
3. Create simple missions (straight lines)
4. Progress to complex missions (turns, patterns)
5. Tune parameters for your specific setup

## Getting Help

- Check logs: `ros2 run rqt_console rqt_console`
- Review README.md for detailed documentation
- Check ArduPilot logs in Mission Planner
