# Warehouse Drone Navigation System

Autonomous indoor navigation system for drones using ArUco markers for localization in warehouse environments.

## Hardware Setup

### Components
- F450 frame
- Hobbywing X-Rotor 40A ESCs
- 2217 950KV BLDC Motors + 10" propellers
- Flysky FS-i6 RC transmitter + FS-iA6B receiver
- Pixhawk 6C flight controller
- Raspberry Pi 4B companion computer
- Logitech C270 webcam
- 3S 5200mAh LiPo battery

### Connections
1. **Pixhawk to RPi**: Connect via USB or serial (TELEM2)
2. **Camera to RPi**: USB connection
3. **ESCs to Pixhawk**: Connect to MAIN OUT ports
4. **RC Receiver to Pixhawk**: Connect to RC IN port
5. **Power**: Connect battery to power distribution board

## Software Stack

### Prerequisites

#### On Raspberry Pi (Ubuntu 22.04)
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop -y

# Install dependencies
sudo apt install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations

# Install Python packages
pip3 install numpy opencv-contrib-python pyyaml

# Install GeographicLib datasets for MAVROS
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

#### ArduPilot Setup on Pixhawk
1. Flash ArduCopter firmware to Pixhawk 6C using Mission Planner
2. Perform initial calibration (compass, accelerometer, radio)
3. Enable vision position estimation:
   ```
   EK3_SRC1_POSXY = 6 (ExternalNav)
   EK3_SRC1_POSZ = 6 (ExternalNav)
   VISO_TYPE = 1 (MAV)
   ```

### Installation

1. **Clone the repository**
   ```bash
   cd ~
   mkdir -p ros2_ws/src
   cd ros2_ws/src
   # Copy the warehouse_drone_nav package here
   ```

2. **Build the package**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select warehouse_drone_nav
   source install/setup.bash
   ```

3. **Make scripts executable**
   ```bash
   cd ~/ros2_ws/src/warehouse_drone_nav/scripts
   chmod +x *.py
   ```

## Configuration

### 1. Camera Calibration
Before using the system, calibrate your camera:

```bash
# Install camera calibration tools
sudo apt install ros-humble-camera-calibration

# Run calibration (print a checkerboard pattern first)
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.024 \
    image:=/camera/image_raw \
    camera:=/camera
```

Save the calibration file and update the camera parameters.

### 2. ArUco Markers
- **Print markers**: Use the DICT_4X4_50 dictionary
- **Marker size**: 15cm x 15cm (0.15m) - adjust in config if different
- **Placement**: Place markers on the ground along the racking paths
- **Marker IDs**: Start with ID 0, 1, 2, 3, 4...

Generate markers using:
```python
import cv2
import cv2.aruco as aruco

# Generate ArUco markers
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
for marker_id in range(5):
    img = aruco.generateImageMarker(aruco_dict, marker_id, 200)
    cv2.imwrite(f'marker_{marker_id}.png', img)
```

### 3. Marker Map
Edit `maps/marker_map_sample.yaml` to define the real-world positions of your markers:

```yaml
markers:
  - id: 0
    position:
      x: 0.0
      y: 0.0
      z: 0.0
  # Add more markers...
```

### 4. Mission Planning
Create a mission file in `maps/` directory defining waypoints:

```yaml
mission_name: "My Mission"
waypoints:
  - x: 0.0
    y: 0.0
    z: 1.5
  - x: 2.0
    y: 0.0
    z: 1.5
  # Add more waypoints...
```

## Usage

### 1. Start MAVROS
In terminal 1:
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
```

### 2. Start Camera Node
In terminal 2:
```bash
# For USB camera
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=/dev/video0 \
    -p image_width:=640 \
    -p image_height:=480 \
    -p pixel_format:=yuyv \
    -p camera_frame_id:=camera_frame \
    -r __ns:=/camera
```

### 3. Test ArUco Detection
In terminal 3:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch warehouse_drone_nav aruco_test.launch.py
```

Check detection visualization:
```bash
# In another terminal
ros2 run rqt_image_view rqt_image_view /aruco/detection_image
```

### 4. Launch Full Navigation System
Once ArUco detection is working:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch warehouse_drone_nav warehouse_nav.launch.py
```

### 5. Start Mission

**Manual Control:**
```bash
# Publish commands to start mission
ros2 topic pub --once /mission/command std_msgs/msg/String "data: 'start'"

# Other commands:
# Pause: ros2 topic pub --once /mission/command std_msgs/msg/String "data: 'pause'"
# Resume: ros2 topic pub --once /mission/command std_msgs/msg/String "data: 'resume'"
# Abort: ros2 topic pub --once /mission/command std_msgs/msg/String "data: 'abort'"
```

**With Mission File:**
```bash
ros2 launch warehouse_drone_nav warehouse_nav.launch.py \
    mission_file:=$(ros2 pkg prefix warehouse_drone_nav)/share/warehouse_drone_nav/maps/mission_sample.yaml \
    auto_start:=true
```

## Monitoring

### View Topics
```bash
# List all topics
ros2 topic list

# Monitor drone position
ros2 topic echo /drone/local_position

# Monitor mission status
ros2 topic echo /mission/status

# Monitor ArUco detections
ros2 topic echo /aruco/poses
```

### View Logs
```bash
# Filter logs by node
ros2 run rqt_console rqt_console
```

## Architecture

```
┌─────────────────┐
│  Logitech C270  │
│     Camera      │
└────────┬────────┘
         │ USB
         ▼
┌─────────────────────────────────────────┐
│         Raspberry Pi 4B                 │
│  ┌──────────────────────────────────┐  │
│  │     ArUco Detector Node          │  │
│  │  - Detects markers               │  │
│  │  - Estimates marker poses        │  │
│  └──────────┬───────────────────────┘  │
│             │                           │
│  ┌──────────▼───────────────────────┐  │
│  │   Position Estimator Node        │  │
│  │  - Fuses marker detections       │  │
│  │  - Publishes to MAVROS           │  │
│  └──────────┬───────────────────────┘  │
│             │                           │
│  ┌──────────▼───────────────────────┐  │
│  │   Flight Controller Node         │  │
│  │  - Sends commands to ArduPilot   │  │
│  │  - Waypoint following            │  │
│  └──────────┬───────────────────────┘  │
│             │                           │
│  ┌──────────▼───────────────────────┐  │
│  │    Mission Manager Node          │  │
│  │  - Mission state machine         │  │
│  │  - High-level coordination       │  │
│  └──────────────────────────────────┘  │
│             │                           │
│  ┌──────────▼───────────────────────┐  │
│  │         MAVROS                   │  │
│  │  - MAVLink bridge                │  │
│  └──────────┬───────────────────────┘  │
└─────────────┼───────────────────────────┘
              │ Serial/USB
              ▼
┌──────────────────────────────┐
│      Pixhawk 6C              │
│      ArduCopter              │
└──────────────────────────────┘
```

## Safety Checklist

Before flight:
- [ ] Battery fully charged
- [ ] All connections secure
- [ ] RC transmitter powered on and bound
- [ ] Sufficient space for safe flight
- [ ] Emergency stop procedure understood
- [ ] ArUco markers clearly visible
- [ ] Camera focused and calibrated
- [ ] MAVROS connected to Pixhawk
- [ ] All ROS nodes running without errors
- [ ] Test mode changes in STABILIZE first

During flight:
- [ ] Monitor battery voltage
- [ ] Keep RC transmitter in hand for manual override
- [ ] Watch for unexpected behavior
- [ ] Be ready to switch to STABILIZE mode

## Troubleshooting

### Camera not detected
```bash
# Check camera device
ls /dev/video*

# Test camera
v4l2-ctl --list-devices
```

### MAVROS not connecting
```bash
# Check serial connection
ls /dev/ttyUSB* /dev/ttyACM*

# Check permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Test connection
sudo mavproxy.py --master=/dev/ttyUSB0 --baudrate=921600
```

### ArUco markers not detected
- Check lighting conditions (avoid glare)
- Verify marker size in config matches physical size
- Ensure camera is in focus
- Check if correct ArUco dictionary is selected
- Verify camera calibration

### Position drift
- Recalibrate camera
- Check marker map coordinates
- Verify marker placement is accurate
- Consider adding more markers for redundancy

### Drone not responding to commands
- Check MAVROS connection
- Verify flight mode (should be GUIDED)
- Check if drone is armed
- Verify position estimate is being published

## Development

### Adding New Features
1. Create new node in `scripts/`
2. Update `CMakeLists.txt` to install script
3. Add configuration in `config/`
4. Update launch file if needed

### Testing
```bash
# Run Python tests
cd ~/ros2_ws
colcon test --packages-select warehouse_drone_nav

# Check code style
flake8 src/warehouse_drone_nav/scripts/
```

## Future Enhancements
- Multi-marker fusion for improved accuracy
- Obstacle avoidance using additional sensors
- Dynamic path planning
- Multiple drone coordination
- Web-based monitoring interface
- Automatic marker detection and mapping

## References
- [ArduPilot Documentation](https://ardupilot.org/copter/)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [OpenCV ArUco Tutorial](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)

## License
MIT License

## Support
For issues and questions, please open an issue on the repository.
