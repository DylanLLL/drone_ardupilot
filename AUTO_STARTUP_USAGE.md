# Auto Startup Node Usage Guide

## Overview

The `auto_startup` node automates the drone startup sequence with two different modes:

- **Bench Test Mode** (default): For testing without propellers
- **Real Flight Mode**: For autonomous flight with automatic takeoff

## Modes Explained

### Bench Test Mode (bench_test_mode=true)

**Use case**: Testing on a bench without propellers attached

**Sequence**:
1. Sets mode to **ALT_HOLD** (altitude hold mode)
2. Arms the drone
3. Skips the takeoff command (since it would fail without propellers)
4. Waits for ArUco tag detection
5. Switches to **GUIDED** mode when tag is detected → Position lock activates

**How to run**:
```bash
ros2 run warehouse_drone_nav auto_startup
# Or explicitly:
ros2 run warehouse_drone_nav auto_startup --ros-args -p bench_test_mode:=true
```

---

### Real Flight Mode (bench_test_mode=false)

**Use case**: Autonomous flight with propellers attached

**Sequence**:
1. Sets mode to **GUIDED_NOGPS** (enables autonomous takeoff without GPS/visual position)
2. Arms the drone
3. **Automatically takes off to 50-60cm altitude** (throttle controlled by flight controller)
4. Monitors altitude until target is reached
5. Waits for ArUco tag detection
6. Switches to **GUIDED** mode when tag is detected → Position lock activates

**How to run**:
```bash
ros2 run warehouse_drone_nav auto_startup --ros-args -p bench_test_mode:=false
```

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bench_test_mode` | `true` | Enable bench test mode (skip takeoff) |
| `target_altitude` | `0.55` | Target takeoff altitude in meters (55cm) |
| `altitude_tolerance` | `0.05` | Altitude tolerance in meters (5cm) |
| `startup_timeout` | `30.0` | Maximum time for sequence in seconds |

### Example with custom parameters:
```bash
ros2 run warehouse_drone_nav auto_startup --ros-args \
  -p bench_test_mode:=false \
  -p target_altitude:=0.60 \
  -p altitude_tolerance:=0.05 \
  -p startup_timeout:=45.0
```

---

## How Autonomous Takeoff Works (Real Flight Mode)

In **GUIDED_NOGPS** mode:
- The flight controller can execute takeoff commands autonomously
- Throttle is automatically controlled to reach the target altitude
- No GPS or visual position estimate is required for takeoff
- Once the drone is airborne and sees the ArUco tag, it switches to **GUIDED** mode for precise position control

This is different from **ALT_HOLD** mode where:
- Throttle must be manually controlled (via RC or manual commands)
- The flight controller only holds altitude after you set it
- Autonomous takeoff commands don't work

---

## Prerequisites

Make sure these nodes are running before starting auto_startup:

```bash
# Terminal 1: ArUco detector
ros2 run warehouse_drone_nav aruco_detector

# Terminal 2: Position estimator
ros2 run warehouse_drone_nav position_estimator

# Terminal 3: Flight controller
ros2 run warehouse_drone_nav flight_controller

# Terminal 4: Auto startup
ros2 run warehouse_drone_nav auto_startup --ros-args -p bench_test_mode:=false
```

---

## Expected Output

### Bench Test Mode:
```
[INFO] [auto_startup]: Starting automated drone startup sequence (BENCH TEST MODE)...
[INFO] [auto_startup]: Using ALT_HOLD mode for bench testing
[INFO] [auto_startup]: Step 1: Setting mode to ALT_HOLD (altitude hold)...
[INFO] [auto_startup]: ALT_HOLD mode set successfully
[INFO] [auto_startup]: Step 2: Arming and taking off to 0.55m altitude...
[INFO] [auto_startup]: Drone armed successfully
[INFO] [auto_startup]: Bench test mode: Skipping takeoff command
[INFO] [auto_startup]: For now, waiting for altitude to reach target or ArUco detection...
[INFO] [auto_startup]: ArUco tag detected! Position estimate available.
[INFO] [auto_startup]: Bench test mode: ArUco detected, skipping altitude check
[INFO] [auto_startup]: ArUco tag detected! Switching from ALT_HOLD to GUIDED mode...
[INFO] [auto_startup]: Successfully switched to GUIDED mode
[INFO] [auto_startup]: Startup sequence complete!
[INFO] [auto_startup]: Drone is now in GUIDED mode with position lock active.
```

### Real Flight Mode:
```
[INFO] [auto_startup]: Starting automated drone startup sequence (REAL FLIGHT MODE)...
[INFO] [auto_startup]: Using GUIDED_NOGPS mode for autonomous takeoff
[INFO] [auto_startup]: Step 1: Setting mode to GUIDED_NOGPS (autonomous takeoff without GPS)...
[INFO] [auto_startup]: GUIDED_NOGPS mode set successfully
[INFO] [auto_startup]: Step 2: Arming and taking off to 0.55m altitude...
[INFO] [auto_startup]: Drone armed successfully
[INFO] [auto_startup]: Takeoff command sent successfully
[INFO] [auto_startup]: Drone is autonomously taking off...
[INFO] [auto_startup]: Current altitude: 0.250m | Target: 0.550m | Error: 0.300m
[INFO] [auto_startup]: Current altitude: 0.450m | Target: 0.550m | Error: 0.100m
[INFO] [auto_startup]: Current altitude: 0.540m | Target: 0.550m | Error: 0.010m
[INFO] [auto_startup]: Target altitude reached! Current: 0.540m
[INFO] [auto_startup]: ArUco tag detected! Position estimate available.
[INFO] [auto_startup]: ArUco tag detected! Switching from GUIDED_NOGPS to GUIDED mode...
[INFO] [auto_startup]: Successfully switched to GUIDED mode
[INFO] [auto_startup]: Startup sequence complete!
[INFO] [auto_startup]: Drone is now in GUIDED mode with position lock active.
```

---

## Troubleshooting

### "Failed to set GUIDED_NOGPS mode"
- Your flight controller may not support GUIDED_NOGPS mode
- Try using ALT_HOLD mode and manually controlling throttle
- Or ensure your ArduPilot/PX4 firmware supports this mode

### "Takeoff command failed"
- In ALT_HOLD mode, takeoff commands don't work (this is expected in bench test)
- In GUIDED_NOGPS mode, ensure the flight controller supports autonomous takeoff
- Check flight controller parameters and pre-arm checks

### ArUco tag not detected
- Ensure camera is running and publishing images
- Check that ArUco markers are visible and properly sized
- Verify `aruco_detector` and `position_estimator` nodes are running

---

## Safety Notes

⚠️ **IMPORTANT**:
- Always test in bench test mode first before real flight
- Ensure proper safety measures when testing with propellers
- Keep a manual RC controller ready for emergency takeover
- Test in a safe, open area away from obstacles
- Start with conservative altitude values (50-60cm recommended)
