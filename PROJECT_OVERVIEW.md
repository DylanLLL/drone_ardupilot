# Warehouse Drone Navigation - Project Overview

## What We've Built

A complete ROS2-based autonomous navigation system for your F450 drone to navigate indoors using ArUco markers. This foundational codebase gives you everything needed to detect markers, estimate position, and command the drone autonomously.

## Project Structure

```
warehouse_drone_nav/
├── src/
│   ├── warehouse_drone_nav/          # Python package
│   │   └── __init__.py
│   ├── scripts/                      # ROS2 nodes (executable)
│   │   ├── aruco_detector_node.py    # ArUco marker detection
│   │   ├── position_estimator_node.py # Position estimation & MAVROS bridge
│   │   ├── flight_controller_node.py  # Flight control interface
│   │   └── mission_manager_node.py    # Mission coordination
│   ├── launch/                       # Launch files
│   │   ├── warehouse_nav.launch.py   # Full system launch
│   │   └── aruco_test.launch.py      # ArUco testing only
│   ├── config/                       # Configuration files
│   │   ├── aruco_detector.yaml
│   │   ├── position_estimator.yaml
│   │   ├── flight_controller.yaml
│   │   └── mission_manager.yaml
│   ├── maps/                         # Marker maps & missions
│   │   ├── marker_map_sample.yaml
│   │   └── mission_sample.yaml
│   ├── resource/                     # ROS2 resources
│   ├── package.xml                   # ROS2 package manifest
│   ├── CMakeLists.txt               # Build configuration
│   └── setup.py                     # Python setup
├── tools/                           # Utility scripts
│   └── generate_markers.py          # Generate ArUco markers
├── README.md                        # Full documentation
├── QUICKSTART.md                    # Quick start guide
└── requirements.txt                 # Python dependencies
```

## Core Components

### 1. ArUco Detector Node (`aruco_detector_node.py`)
**Purpose**: Detects ArUco markers from camera feed and estimates their poses

**Key Features**:
- Real-time marker detection using OpenCV
- Pose estimation for each detected marker
- Visualization of detections
- Publishes marker poses to ROS2 topics

**Topics Published**:
- `/aruco/poses` - Array of detected marker poses
- `/aruco/detection_image` - Visualization image

**Configuration**: `config/aruco_detector.yaml`

### 2. Position Estimator Node (`position_estimator_node.py`)
**Purpose**: Converts ArUco detections to global position estimates and publishes to ArduPilot

**Key Features**:
- Transforms camera-relative detections to world frame
- Marker map support for known marker positions
- Publishes to MAVROS for ArduPilot integration
- TF broadcasting for coordinate transforms

**Topics Published**:
- `/mavros/vision_pose/pose` - Vision position for ArduPilot
- `/drone/local_position` - Estimated drone position
- `/drone/odometry` - Odometry data

**Topics Subscribed**:
- `/aruco/poses` - ArUco detections
- `/mavros/state` - MAVROS connection state

**Configuration**: `config/position_estimator.yaml`

### 3. Flight Controller Node (`flight_controller_node.py`)
**Purpose**: High-level flight control interface with ArduPilot via MAVROS

**Key Features**:
- Arm/disarm commands
- Takeoff/land commands
- Position setpoint control
- Mode switching (GUIDED, STABILIZE, etc.)
- Waypoint following

**Topics Published**:
- `/mavros/setpoint_position/local` - Position setpoints

**Topics Subscribed**:
- `/drone/local_position` - Current position
- `/drone/command` - High-level commands
- `/mavros/state` - Flight controller state

**Services Used**:
- `/mavros/cmd/arming` - Arm/disarm
- `/mavros/set_mode` - Mode changes
- `/mavros/cmd/takeoff` - Takeoff
- `/mavros/cmd/land` - Landing

**Configuration**: `config/flight_controller.yaml`

### 4. Mission Manager Node (`mission_manager_node.py`)
**Purpose**: Coordinates autonomous navigation missions

**Key Features**:
- Mission state machine (IDLE, TAKEOFF, NAVIGATING, LANDING, etc.)
- Waypoint management
- Mission loading from YAML files
- Mission control (start, pause, resume, abort)

**Topics Published**:
- `/drone/command` - Commands to flight controller
- `/mission/status` - Mission status updates

**Topics Subscribed**:
- `/drone/local_position` - Drone position
- `/mission/command` - Mission commands

**Configuration**: `config/mission_manager.yaml`

## Data Flow

```
Camera → ArUco Detector → Position Estimator → MAVROS → ArduPilot/Pixhawk
                              ↓
                         Flight Controller ← Mission Manager
```

1. **Camera** captures images
2. **ArUco Detector** finds markers and estimates poses
3. **Position Estimator** converts to global position and sends to MAVROS
4. **MAVROS** forwards vision position to ArduPilot
5. **ArduPilot** uses vision position for localization
6. **Mission Manager** coordinates waypoint navigation
7. **Flight Controller** sends commands to ArduPilot

## Next Steps: Getting ArUco Detection Working

### Phase 1: Hardware Setup (Do First!)
1. Mount Logitech C270 camera on drone facing downward
2. Connect camera to Raspberry Pi via USB
3. Connect Pixhawk to RPi via USB/serial
4. Verify connections

### Phase 2: Generate and Print Markers
```bash
# Generate markers
cd warehouse_drone_nav/tools
python3 generate_markers.py --output-dir ~/aruco_markers --start-id 0 --end-id 4

# Print markers (on your computer with a printer)
# Each marker should be 15cm x 15cm when printed
# Place markers on the ground in known positions
```

### Phase 3: Camera Calibration
```bash
# Install calibration tools
sudo apt install ros-humble-camera-calibration

# Print checkerboard pattern (8x6, 24mm squares)
# Run calibration
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.024 \
    image:=/camera/image_raw \
    camera:=/camera

# Save calibration when done
```

### Phase 4: Test ArUco Detection
```bash
# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=/dev/video0 \
    -r __ns:=/camera

# Terminal 2: Start ArUco detector
ros2 launch warehouse_drone_nav aruco_test.launch.py

# Terminal 3: View detections
ros2 run rqt_image_view rqt_image_view /aruco/detection_image
```

**Verify**:
- Markers are detected (green outlines)
- Axes are drawn on markers
- Console shows detection messages
- Adjust camera height/angle for best results

### Phase 5: Configure Marker Map
1. Measure physical positions of your markers
2. Edit `maps/marker_map_sample.yaml` with actual coordinates
3. Update `config/position_estimator.yaml` to use your map file

### Phase 6: Test Position Estimation
```bash
# Launch full system
ros2 launch warehouse_drone_nav warehouse_nav.launch.py

# Monitor position estimates
ros2 topic echo /drone/local_position

# Verify position updates as drone sees different markers
```

### Phase 7: Configure ArduPilot
In Mission Planner, set these parameters:
```
EK3_SRC1_POSXY = 6 (ExternalNav)
EK3_SRC1_POSZ = 6 (ExternalNav) 
VISO_TYPE = 1 (MAV)
```

### Phase 8: Test Integration with MAVROS
```bash
# Terminal 1: MAVROS
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600

# Terminal 2: Camera
ros2 run usb_cam usb_cam_node_exe --ros-args -r __ns:=/camera

# Terminal 3: Navigation system
ros2 launch warehouse_drone_nav warehouse_nav.launch.py

# Terminal 4: Monitor
ros2 topic echo /mavros/vision_pose/pose
```

**Verify**:
- MAVROS connected to Pixhawk
- Vision pose being published
- ArduPilot receiving vision data (check in Mission Planner)

## Testing Strategy

### Test 1: Static Detection (No Flight)
- Place markers on ground
- Hold drone over markers at different heights
- Verify consistent detection
- Check position estimates

### Test 2: Manual Flight + Vision
- Fly in STABILIZE mode manually
- Monitor position estimates
- Verify accuracy and stability

### Test 3: Guided Mode Hover
- Switch to GUIDED mode
- Command hover at current position
- Verify drone holds position using vision

### Test 4: Simple Waypoint
- Create mission with 1-2 waypoints
- Test autonomous navigation
- Monitor closely, ready for manual override

### Test 5: Full Mission
- Run complete racking navigation
- Monitor performance
- Tune parameters as needed

## Key Parameters to Tune

### ArUco Detection
- `marker_size` - Must match physical marker size exactly
- `camera_topic` - Your camera's topic name
- `aruco_dict_type` - Match the dictionary you used to generate markers

### Position Estimation  
- `marker_map_file` - Path to your marker positions
- `use_vision_position` - Enable/disable publishing to MAVROS

### Flight Control
- `default_altitude` - Flight height above ground
- `max_velocity` - Speed limit for safety
- `position_tolerance` - How close to waypoint = reached

### Mission
- `waypoint_tolerance` - Distance to waypoint for "reached"
- `auto_start` - Whether to start mission on launch

## Common Issues and Solutions

### Markers Not Detected
- **Check lighting** - Avoid glare, shadows
- **Verify marker size** in config matches physical size
- **Camera focus** - Make sure it's focused at flight height
- **Correct dictionary** - DICT_4X4_50 by default

### Position Jumps/Unstable
- **Recalibrate camera** - Poor calibration causes issues
- **Check marker map** - Verify coordinates are correct
- **Add more markers** - More markers = better localization
- **Marker quality** - Ensure markers are flat, clean, well-lit

### MAVROS Not Connecting
- **Check USB device** - `/dev/ttyUSB0` or `/dev/ttyACM0`
- **Verify baud rate** - 921600 for Pixhawk 6C
- **Check permissions** - User must be in `dialout` group

### Drone Not Responding
- **Verify GUIDED mode** - Must be in GUIDED for autonomous control
- **Check arming** - Drone must be armed
- **Monitor topics** - Ensure commands are being published

## Safety Reminders

⚠️ **ALWAYS** have RC transmitter in hand for manual override
⚠️ **ALWAYS** test in STABILIZE mode first
⚠️ **ALWAYS** have clear emergency stop procedure
⚠️ **ALWAYS** ensure adequate space for safe flight
⚠️ **ALWAYS** monitor battery voltage

## Resources

- **Full Documentation**: `README.md`
- **Quick Start**: `QUICKSTART.md`
- **ArduPilot Docs**: https://ardupilot.org/copter/
- **MAVROS Docs**: http://wiki.ros.org/mavros
- **ROS2 Docs**: https://docs.ros.org/en/humble/

## Support

This is a foundational codebase to get you started. As you test and develop:
1. Monitor logs for issues
2. Tune parameters for your specific setup  
3. Add features as needed
4. Start simple and build complexity gradually

Good luck with your autonomous warehouse drone! 🚁
