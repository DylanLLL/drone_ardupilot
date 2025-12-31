# Vision-Based Position Hold Testing Guide

## Overview
This guide provides step-by-step procedures for testing vision-based position hold using ArUco markers with manual altitude control.

## Test Scenario
1. Manually fly drone in **ALT_HOLD** mode using RC transmitter
2. Position drone at height where camera can see ArUco markers (1-2 meters)
3. Switch to **GUIDED** mode via RC transmitter
4. System automatically holds X, Y, Z position using vision feedback

---

## Prerequisites

### Hardware Setup
- ✅ Pixhawk 6C flight controller connected via USB
- ✅ Logitech C270 camera connected and working
- ✅ ArUco markers (DICT_4X4_50) printed and positioned
- ✅ RC transmitter configured with mode switch
- ✅ Battery fully charged
- ✅ Propellers attached (for actual flight test)

### ArduPilot Configuration
Ensure these parameters are set (use Mission Planner or MAVProxy):

```
EK3_SRC1_POSXY = 6   (ExternalNav)
EK3_SRC1_POSZ = 6    (ExternalNav)
EK3_SRC1_VELXY = 6   (ExternalNav)
EK3_SRC1_VELZ = 6    (ExternalNav)
EK3_SRC1_YAW = 1     (Compass)
VISO_TYPE = 1        (MAV)
GPS_TYPE = 0         (None - GPS disabled)
```

### RC Transmitter Setup
- **Channel 5 or 6**: Flight mode switch
  - Position 1: **STABILIZE** (manual control)
  - Position 2: **ALT_HOLD** (altitude hold, manual X/Y)
  - Position 3: **GUIDED** (full autonomous control)
- **Throttle stick**: Altitude control in ALT_HOLD
- **Roll/Pitch**: Horizontal position control

---

## Pre-Flight Checklist

### 1. System Startup

**Terminal 1: Launch MAVROS**
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
```

Wait for:
```
[INFO] [mavros]: MAVROS started
[INFO] [mavros]: FCU: ArduPilot
[INFO] [mavros]: IMU: Detected!
```

**Terminal 2: Launch Camera**
```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0 \
  -p pixel_format:=yuyv
```

**Terminal 3: Launch Navigation System**
```bash
cd /home/gdnuser/warehouse_drone_nav
ros2 launch src/launch/warehouse_nav.launch.py
```

Expected output:
```
[flight_controller] [INFO] Flight Controller Node initialized
[flight_controller] [INFO] MAVROS connected!
[aruco_detector] [INFO] ArUco Detector Node initialized
[position_estimator] [INFO] Position Estimator Node initialized
```

### 2. Verify Vision Position

**Terminal 4: Check Vision Position Publishing**
```bash
ros2 topic echo /mavros/vision_pose/pose
```

You should see pose data updating at ~30 Hz when ArUco markers are visible.

**Terminal 5: Check Drone Position Estimate**
```bash
ros2 topic echo /drone/local_position
```

**Terminal 6: Monitor ArUco Detections**
```bash
ros2 topic echo /aruco/poses
```

When markers are visible, you should see marker ID and position data.

### 3. Pre-Flight Safety Checks

- [ ] RC transmitter powered ON
- [ ] Flight mode switch in **STABILIZE** position
- [ ] Battery voltage > 11.1V (for 3S LiPo)
- [ ] Propellers securely attached
- [ ] Flight area clear (minimum 3m x 3m)
- [ ] ArUco markers visible and properly positioned
- [ ] Emergency landing area identified
- [ ] All ROS2 nodes running successfully

---

## Flight Test Procedure

### Phase 1: Manual Takeoff in ALT_HOLD

1. **Arm the drone** using RC transmitter stick command:
   - Throttle: DOWN
   - Rudder: RIGHT
   - Hold for 2 seconds
   - Motors should spin up

2. **Switch to ALT_HOLD mode** using mode switch

3. **Takeoff manually**:
   - Gradually increase throttle
   - Lift off smoothly
   - Climb to 1.5-2.0 meters altitude
   - Stabilize hover using roll/pitch sticks

4. **Position drone** so camera can see ArUco markers:
   - Look at monitor showing camera feed (if available)
   - OR check terminal output: `ros2 topic echo /aruco/poses`
   - When markers detected, you should see distance readings

### Phase 2: Verify Vision Position Lock

**Terminal 7: Monitor Position Estimate**
```bash
ros2 topic echo /drone/local_position
```

You should see stable X, Y, Z coordinates when markers are detected.

**Check pose updates**:
```bash
ros2 topic hz /mavros/vision_pose/pose
```

Should show ~30 Hz update rate.

### Phase 3: Enable GUIDED Mode (Position Hold)

1. **Verify drone is stable** in ALT_HOLD hover

2. **Check vision position** is publishing (Terminal 7 showing stable values)

3. **Switch to GUIDED mode** using RC mode switch

4. **Expected behavior**:
   - Flight controller logs: `Switched to GUIDED mode - enabling position hold`
   - Flight controller logs: `Holding position: X=... Y=... Z=...`
   - Drone should **actively hold position** using vision feedback
   - Small drifts should be corrected automatically

5. **Observe position hold**:
   - Drone maintains X, Y coordinates
   - Drone maintains Z (altitude)
   - Gentle external pushes should be corrected

### Phase 4: Test Position Hold Stability

**Monitor position error**:
```bash
# Watch how close drone stays to target
ros2 topic echo /drone/local_position
```

**Test scenarios**:
1. **No external disturbance** - Should hold rock-steady
2. **Gentle push** - Should return to hold position
3. **Partial marker occlusion** - Should maintain last known position
4. **Full marker visibility restored** - Should re-lock position

### Phase 5: Manual Position Hold Command (Optional)

While in GUIDED mode, you can also manually trigger position hold:

```bash
ros2 topic pub --once /drone/command std_msgs/msg/String "data: 'hold'"
```

This captures current position and holds it.

### Phase 6: Return to Manual Control

1. **Switch back to ALT_HOLD** using RC mode switch
   - Drone returns to manual altitude hold control
   - You regain roll/pitch control

2. **Land manually**:
   - Reduce throttle gradually
   - Touch down gently
   - Throttle to minimum

3. **Disarm** using stick command:
   - Throttle: DOWN
   - Rudder: LEFT
   - Hold for 2 seconds

---

## Expected Behavior in GUIDED Mode

### ✅ Correct Behavior
- Drone holds position within ±0.15m (configurable tolerance)
- Small oscillations are normal due to vision update rate
- Drone corrects drift automatically
- Position setpoints published at 20 Hz to `/mavros/setpoint_position/local`

### ⚠️ Warning Signs
- Large oscillations (> 0.5m) - Check PID tuning or vision quality
- Drifting in one direction - Marker map may be incorrect
- Sudden jumps - Marker ID confusion or bad detection

### ❌ Abort Conditions
- Drone flies away uncontrollably → Switch to STABILIZE immediately
- Vision position loss → Will fall back to last known position
- Loss of RC link → RTL mode will activate (if configured)

---

## Troubleshooting

### Issue: Drone doesn't hold position in GUIDED
**Check:**
```bash
# Is target position being set?
ros2 topic echo /mavros/setpoint_position/local

# Is vision position publishing?
ros2 topic hz /mavros/vision_pose/pose

# Are ArUco markers detected?
ros2 topic echo /aruco/poses
```

**Solution:**
- Ensure markers are visible
- Check lighting conditions
- Verify marker map is loaded correctly

### Issue: Position oscillates wildly
**Possible causes:**
- Vision position noise
- Incorrect marker map coordinates
- PID tuning needed in ArduPilot

**Solution:**
- Increase `min_marker_confidence` in [config/position_estimator.yaml](src/config/position_estimator.yaml)
- Verify marker positions in marker map file
- Tune GUIDED mode PIDs in ArduPilot

### Issue: Drone drifts slowly
**Check:**
```bash
# Monitor position estimate stability
ros2 topic echo /drone/local_position
```

**Solution:**
- Marker map coordinates may be inaccurate
- Re-measure marker positions
- Increase number of visible markers

### Issue: "Cannot start position hold - no position estimate"
**This means no ArUco markers are detected**

**Solution:**
1. Check camera feed: `ros2 topic hz /camera/image_raw`
2. Check marker detection: `ros2 topic echo /aruco/poses`
3. Ensure markers are visible to camera
4. Check lighting (avoid glare/shadows)
5. Verify camera is pointing at markers

---

## Manual Commands Reference

All commands sent via:
```bash
ros2 topic pub --once /drone/command std_msgs/msg/String "data: 'COMMAND'"
```

Available commands:
- `arm` - Arm the drone
- `disarm` - Disarm the drone
- `hold` - Hold current position
- `goto X Y Z` - Go to specific coordinates
- `land` - Initiate landing
- `takeoff` - Autonomous takeoff (NOT recommended for this test scenario)

---

## Diagnosing Position Lock Behavior

### Using the Position Monitor Tool

The position monitor provides real-time visualization of position errors:

```bash
# Launch position monitor
cd /home/gdnuser/warehouse_drone_nav
python3 src/scripts/position_monitor.py
```

**What you'll see**:
```
══════════════════════════════════════════════════════════════════════
  POSITION HOLD MONITOR
══════════════════════════════════════════════════════════════════════

  Current Position:    X: +0.045m  Y: -0.032m  Z: +0.605m
  Target Position:     X: +0.000m  Y: +0.000m  Z: +0.600m
  ──────────────────────────────────────────────────────────────────
  Position Error:      X: -0.045m →  Y: +0.032m ↑  Z: -0.005m ↓

  Distance from target: 0.056m
  Status: ⚠ CORRECTING

  Corrections needed:
    ← Move LEFT   0.045m
    ↓ Move BACK   0.032m
    ↓ Move DOWN   0.005m
══════════════════════════════════════════════════════════════════════
```

**How to interpret**:
- **Negative X error (-0.045m)**: Drone is too far RIGHT → should move LEFT
- **Positive Y error (+0.032m)**: Drone is too far BACK → should move FORWARD
- **Negative Z error (-0.005m)**: Drone is too HIGH → should move DOWN

### Bench Testing Position Response

**Test 1: Verify Vision Updates**
1. Arm drone in ALT_HOLD, hold where markers visible
2. Launch position monitor
3. Manually move drone RIGHT (10cm)
4. **Expected**: X value INCREASES in "Current Position"
5. Move drone FORWARD (10cm)
6. **Expected**: Y value INCREASES

**Test 2: Verify Correction Direction**
1. With drone armed in GUIDED mode
2. Position monitor shows target position
3. Manually move drone RIGHT (10cm)
4. **Expected**:
   - Position Error shows: `X: -0.100m` (negative = too far right)
   - Corrections show: `← Move LEFT  0.100m`
5. Watch PWM outputs in Mission Planner:
   - RIGHT motors (1 & 4) should INCREASE PWM
   - LEFT motors (2 & 3) should DECREASE PWM

This confirms the flight controller is trying to correct in the RIGHT direction!

---

## Fixing PWM Oscillations (CRITICAL)

### Problem: Large PWM Oscillations (±200 PWM)

If you see large, rapid PWM oscillations when in GUIDED mode, **DO NOT FLY**. This indicates the position controller is too aggressive and will cause instability in flight.

### Root Cause

ArduPilot's default PID gains are tuned for GPS-based navigation with slow updates (~5-10 Hz). Vision-based navigation updates much faster (~30 Hz) and can cause oscillations if gains are too high.

### Solution: Reduce ArduPilot PID Gains

**Using Mission Planner:**

1. Connect to Pixhawk via Mission Planner
2. Go to **CONFIG → Full Parameter List**
3. Set these parameters:

```
# Position Controller (reduce by 50-70%)
PSC_POSXY_P = 0.3      # Default: ~1.0
PSC_POSZ_P = 0.3       # Default: ~1.0

# Velocity Controller (reduce by 40-50%)
PSC_VELXY_P = 0.8      # Default: ~2.0
PSC_VELZ_P = 1.5       # Default: ~4.0

# Add Low-Pass Filtering
PSC_VELXY_FILT_HZ = 2.0   # Filter fast changes
PSC_VELZ_FILT_HZ = 2.0    # Filter fast changes

# Limit Accelerations
PSC_ACC_XY_FILT = 10.0    # Smoother horizontal response
PILOT_ACCEL_Z = 150       # Smoother vertical response
```

4. Click **Write Params**
5. **Reboot** the flight controller

### Verification After Tuning

1. Bench test in GUIDED mode again
2. Monitor PWM outputs
3. **Expected**: PWM changes should be ±50 or less (smooth, gradual)
4. If still oscillating, reduce PSC_POSXY_P and PSC_POSZ_P further (try 0.2)

### Progressive Tuning Approach

Start very conservative and increase gradually:

**Step 1 - Very Soft** (start here):
```
PSC_POSXY_P = 0.2
PSC_VELXY_P = 0.5
```

**Step 2 - Soft** (if too sluggish):
```
PSC_POSXY_P = 0.3
PSC_VELXY_P = 0.8
```

**Step 3 - Moderate** (if stable, can try):
```
PSC_POSXY_P = 0.5
PSC_VELXY_P = 1.2
```

**Goal**: Smooth, stable response without oscillations

---

## ROS2 Configuration Improvements

The flight controller has been updated with oscillation reduction features:

**In [src/config/flight_controller.yaml](src/config/flight_controller.yaml)**:
```yaml
control_rate: 10.0           # Reduced from 20.0 Hz for gentler control
position_deadband: 0.05      # Ignore errors <5cm to reduce jitter
```

**Position Error Logging**: The flight controller now logs position errors every 2 seconds:
```
[flight_controller] Position Error: X=-0.045m Y=+0.032m Z=-0.005m | Distance=0.056m
```

This helps you verify that corrections are being calculated correctly.

---

## Monitoring Commands

### Check MAVROS State
```bash
ros2 topic echo /mavros/state
```

### Check Vision Position
```bash
ros2 topic echo /mavros/vision_pose/pose
```

### Check Drone Position
```bash
ros2 topic echo /drone/local_position
```

### Check Setpoint Being Sent
```bash
ros2 topic echo /mavros/setpoint_position/local
```

### Check ArUco Detections
```bash
ros2 topic echo /aruco/poses
```

### Monitor All Nodes
```bash
ros2 node list
```

---

## Safety Procedures

### Emergency Landing
**Method 1: RC Override (Recommended)**
- Switch to **STABILIZE** mode immediately
- Land manually with throttle control

**Method 2: ROS2 Command**
```bash
ros2 topic pub --once /drone/command std_msgs/msg/String "data: 'land'"
```

**Method 3: Direct MAVROS**
```bash
ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{}"
```

### Emergency Disarm (ON GROUND ONLY!)
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

⚠️ **Never disarm while airborne** - drone will drop immediately!

---

## Post-Flight Analysis

### Review Logs
```bash
# Check flight controller logs
ros2 run rqt_console rqt_console

# Review position hold performance
ros2 bag record /drone/local_position /mavros/setpoint_position/local
```

### Success Criteria
- ✅ Drone held position within tolerance (±0.15m)
- ✅ Vision position updated consistently (30 Hz)
- ✅ No crashes or loss of control
- ✅ Smooth transition between modes
- ✅ Stable hover for at least 30 seconds

---

## Next Steps After Successful Test

1. ✅ **Fine-tune position tolerance** - Adjust in [config/flight_controller.yaml](src/config/flight_controller.yaml)
2. ✅ **Test waypoint navigation** - Send `goto` commands
3. ✅ **Test full autonomous mission** - Use mission manager
4. ✅ **Optimize marker placement** - Add more markers for better coverage
5. ✅ **Tune PID parameters** - Improve position hold stability

---

## Configuration Files Reference

- **Flight Controller**: [src/config/flight_controller.yaml](src/config/flight_controller.yaml)
- **Position Estimator**: [src/config/position_estimator.yaml](src/config/position_estimator.yaml)
- **ArUco Detector**: [src/config/aruco_detector.yaml](src/config/aruco_detector.yaml)
- **Marker Map**: [src/maps/marker_map_sample.yaml](src/maps/marker_map_sample.yaml)

---

## Summary

This test validates:
1. ✅ Vision-based position estimation from ArUco markers
2. ✅ MAVROS integration with ArduPilot
3. ✅ Automatic position hold when entering GUIDED mode
4. ✅ Manual-to-autonomous transition workflow
5. ✅ RC override safety

**Key Success**: Drone maintains stable hover in GUIDED mode using only vision feedback from ArUco markers!
