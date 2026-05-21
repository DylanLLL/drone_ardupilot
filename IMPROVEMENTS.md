# Position Hold Improvements — Change Log & Tuning Guide

**Branch:** `feature/improved-position-hold`  
**Base commit:** `a2c42d2` (add moving average to filter out any noisy vision data)  
**Date:** 2026-05-21  
**Author:** yosvas (assisted by Claude Sonnet 4.6)

---

## Problem Statement

When the drone hovers in GUIDED mode and is commanded to hold position, it continuously drifts left or right and requires constant manual RC input to keep the ArUco marker visible under the downward-facing camera. The drift was not a hardware or calibration issue — it was caused by three compounding software bugs.

---

## Root Causes Found

### Bug 1 — Yaw was completely ignored in position estimation (CRITICAL)

**File:** `src/scripts/position_estimator_node.py`  
**Function:** `_estimate_drone_position_from_marker()`

The original code computed the drone's world-frame position with a formula that assumed the drone **always faces exactly 0° yaw** (perfectly aligned with the marker axes):

```python
# ORIGINAL CODE (wrong when drone has any yaw)
offset_x_world = -(-cam_y)   # = cam_y
offset_y_world = -(cam_x)    # = -cam_x
drone_orientation = (0.0, 0.0, 0.0, 1.0)  # hardcoded identity — yaw always = 0°
```

**Why this causes drift:**  
The camera sees the marker at position `(cam_x, cam_y)` in image space. To convert this to world-frame coordinates, you must rotate by the drone's current yaw. If the drone yaws even 10°, the X and Y offsets fed into ArduPilot's EKF are wrong. ArduPilot then tries to correct in the wrong direction, which causes further yaw and further position error — a runaway feedback loop.

**Additionally:** The method `_get_camera_to_body_transform()` defined the correct rotation matrix for this exact purpose but was **never called anywhere**. The inline math and the unused method contradicted each other.

---

### Bug 2 — EMA filter lag caused ArduPilot to overcorrect (HIGH)

**File:** `src/scripts/position_estimator_node.py`  
**Parameter:** `position_filter_alpha = 0.3`

With alpha = 0.3, each new reading contributes only 30% of the filtered output. This creates significant temporal lag between the drone's actual position and what ArduPilot sees. When the drone drifts, ArduPilot acts on the lagged estimate and overcorrects — the drone swings past the target, then corrects again in the other direction. This produces the characteristic left-right oscillation.

---

### Bug 3 — No Python-side PID; ArduPilot's GPS-tuned loop used for vision (HIGH)

**File:** `src/scripts/flight_controller_node.py`  
**Function:** `control_loop()`

The original control loop published a static `PoseStamped` position target:

```python
# ORIGINAL CODE — publishes a fixed target; relies entirely on ArduPilot's internal loop
if distance > self.position_deadband:
    self.setpoint_position_pub.publish(self.target_pose)
```

ArduPilot's position hold PID gains (`PSC_POSXY_P`, `PSC_VELXY_P/I/D`) are tuned for GPS, which provides smooth 5–10 Hz position updates. Vision provides 30 Hz updates with measurement noise and EMA lag. The GPS-tuned gains produce aggressive corrections that amplify vision noise rather than dampening it.

---

## Changes Made

### `src/scripts/position_estimator_node.py`

#### 1. Yaw-corrected position estimation (replaces `_estimate_drone_position_from_marker`)

The original broken formula is replaced with the mathematically correct inverse camera projection.

**The math:**  
ArUco's `estimatePoseSingleMarkers` returns `R_cam_marker`, a rotation matrix that transforms vectors from the marker's coordinate frame into the camera's coordinate frame. The marker is known to be on the ground; the camera is above it. Therefore:

```
cam_translation = R_cam_marker @ (marker_world - drone_world)
```

Rearranging:

```
drone_world = marker_world - R_cam_marker^T @ cam_translation
```

This formula is **always correct regardless of drone yaw**, because `R_cam_marker^T` automatically accounts for the drone's orientation at the time of detection.

Yaw is then extracted by chaining the camera-to-world rotation with the fixed camera-body mount rotation:

```python
R_body_cam = [[0, -1, 0],   # Camera +X (image right) = Body +Y
              [1,  0, 0],   # Camera +Y (image down)  = Body -X (backward)
              [0,  0, 1]]   # Camera +Z (depth)        = Body +Z (down)

R_world_body = R_cam_marker.T @ R_body_cam.T
yaw = atan2(R_world_body[1, 0], R_world_body[0, 0])
```

**Camera mount assumption:** Camera is mounted facing straight down, with the top of the image pointing toward the drone nose. If your camera is rotated differently (e.g., 90° clockwise relative to the nose), `R_body_cam` must be adjusted accordingly.

#### 2. Circular EMA filter for yaw (added to `aruco_callback`)

Averaging yaw angles naively (e.g., `0.6 * new_yaw + 0.4 * old_yaw`) breaks at the ±180° boundary — the average of +179° and −179° should be ±180°, not 0°. The fix uses separate sin/cos components:

```python
filtered_yaw_sin = alpha * sin(new_yaw) + (1-alpha) * filtered_yaw_sin
filtered_yaw_cos = alpha * cos(new_yaw) + (1-alpha) * filtered_yaw_cos
filtered_yaw = atan2(filtered_yaw_sin, filtered_yaw_cos)
```

#### 3. EMA alpha raised from 0.3 → 0.6

More responsive to actual position changes; reduces the lag that was causing ArduPilot to overcorrect. Configurable via `position_estimator.yaml`.

#### 4. Removed dead code

`_get_camera_to_body_transform()` was defined but never called. Its logic is now correctly used inside `_estimate_drone_position_from_marker()` via the inline `R_body_cam` matrix.

---

### `src/scripts/flight_controller_node.py`

#### 5. Python-side XY velocity PID (replaces the static position setpoint loop)

Instead of publishing a fixed position target and relying on ArduPilot's GPS-tuned loop, we now run our own PID at 20 Hz and publish **velocity commands** via `/mavros/setpoint_raw/local` using the `PositionTarget` message type.

The `PositionTarget` is configured as:
- **XY:** velocity setpoints (PID output)
- **Z:** position setpoint (altitude hold — unchanged from before)
- **Yaw:** fixed at 0° (hold current heading)

This bypasses ArduPilot's position P loop and gives us full control over the correction velocity, with gains tuned for vision rather than GPS.

**Anti-windup:** The integral term is clamped to ±0.5 m·s. Without this, the integrator can wind up during periods of large sustained error (e.g., during takeoff before vision locks) and produce a large initial velocity burst that causes the drone to shoot sideways.

**PID resets:** The integral and derivative state are reset to zero when:
- The drone leaves GUIDED mode (mode switch)
- A new position target is set (`_start_position_hold()` or `goto_position()`)

This prevents stale integral values from one hold position corrupting a new hold.

---

### `src/config/flight_controller.yaml`

Added PID gain parameters — all tunable at launch without recompiling:

```yaml
pid_kp: 0.4
pid_ki: 0.05
pid_kd: 0.1
pid_max_vel: 0.5       # m/s cap per axis
pid_integral_max: 0.5  # m·s anti-windup clamp
```

---

### `src/config/position_estimator.yaml`

Added `position_filter_alpha: 0.6` as an explicit configurable parameter.

---

## PID Tuning Guide

### Before you tune

1. Verify the yaw fix is working: watch the `Yaw_raw=` log output from `position_estimator_node`. As you slowly rotate the drone by hand (motors off, camera pointing at marker), the yaw value should change smoothly and return to the same value when you return to the original heading.

2. Set `pid_ki: 0.0` and `pid_kd: 0.0` initially. Tune `pid_kp` alone first.

3. Observe the `PID | Error:` and `Vel_cmd:` log lines in `flight_controller_node` to see what the controller is doing.

---

### Step 1 — Tune Kp (proportional gain)

| Observation | Action |
|---|---|
| Drone barely corrects when pushed; drifts slowly back | Increase `pid_kp` by 0.1 |
| Drone corrects but overshoots and oscillates continuously | Decrease `pid_kp` by 0.1 |
| Drone corrects, slightly overshoots once, then settles | Good — move to Step 2 |

**Typical good range:** `pid_kp = 0.3 – 0.6`

---

### Step 2 — Tune Kd (derivative gain)

Derivative dampens oscillations. Add it if Step 1 left any residual oscillation.

| Observation | Action |
|---|---|
| Drone still oscillates after Kp is set | Increase `pid_kd` by 0.05 |
| Drone response becomes jerky or twitchy | Decrease `pid_kd` by 0.05 |
| Drone settles smoothly without overshoot | Good — move to Step 3 |

**Typical good range:** `pid_kd = 0.05 – 0.2`

> **Warning:** High Kd amplifies noise in the position estimate. If the filtered position has visible jitter, reduce `pid_kd` or increase `position_filter_alpha` (more smoothing).

---

### Step 3 — Tune Ki (integral gain)

Integral eliminates steady-state offset. Only add it if the drone settles near — but not exactly at — the target with a consistent small offset.

| Observation | Action |
|---|---|
| Drone holds ~10–20 cm away from target consistently | Increase `pid_ki` by 0.01 |
| Drone slowly drifts away over 10–20 seconds | Increase `pid_ki` by 0.02 |
| Drone starts oscillating slowly (integral wind-up) | Decrease `pid_ki` or reduce `pid_integral_max` |

**Typical good range:** `pid_ki = 0.02 – 0.1`

> **Warning:** Never increase `pid_ki` without first getting `pid_kp` right. An overly large Ki with a poorly tuned Kp causes slow growing oscillations that are hard to diagnose.

---

### Step 4 — Tune EMA alpha

The `position_filter_alpha` in `position_estimator.yaml` trades off responsiveness vs. noise:

| Alpha | Effect |
|---|---|
| 0.3 | Heavy smoothing, 300–400 ms lag — caused the original overcorrection |
| 0.6 | Moderate smoothing, ~100 ms lag — good starting point |
| 0.8 | Light smoothing, ~50 ms lag — use if Kd is well tuned to handle noise |

Increase alpha if the drone responds sluggishly to disturbances. Decrease alpha if the velocity commands are noisy (visible jitter in `Vel_cmd:` logs).

---

### ArduPilot parameters to verify (Mission Planner)

These ArduPilot parameters should be set for vision-based flight. Check them if position hold still misbehaves after tuning the Python PID:

| Parameter | Recommended | Reason |
|---|---|---|
| `EK2_GPS_TYPE` | `3` | Disable GPS, use external nav (vision) only |
| `EK2_POSNE_M_NSE` | `0.1` | Tell EKF to trust vision (lower = trust more) |
| `EK2_VELN_M_NSE` | `0.1` | Vision velocity noise |
| `PSC_POSXY_P` | `0.5` | Reduce from GPS default ~1.0 |
| `PSC_VELXY_P` | `1.0` | Reduce from GPS default ~2.0 |
| `PSC_VELXY_I` | `0.5` | |
| `PSC_VELXY_D` | `0.2` | |

---

## How to Switch Between Versions

### Use the improved version (this branch)
```bash
git checkout feature/improved-position-hold
```

### Revert to the original version (main branch)
```bash
git checkout main
```

### On the drone (Raspberry Pi), after switching branches
```bash
cd ~/ros2_ws
colcon build --packages-select warehouse_drone_nav
source install/setup.bash
```

---

## Known Limitations Not Yet Fixed

1. **Single-marker ID assumption** — `aruco_callback` always uses the first key in the marker map regardless of which physical marker is actually detected. For a single-marker setup this is fine; for multi-marker setups, a custom ROS message type that carries the marker ID alongside the pose is needed.

2. **Distance scale factor hardcoded** — `aruco_detector_node.py` has `self.distance_scale_factor = 0.632` hardcoded on line 42, ignoring the YAML parameter. This shrinks all position measurements (XYZ) by 36.8%. Verify this value is correct for your specific camera and lens, or set it to `1.0` if camera calibration is done properly.

3. **Marker orientation assumption** — The yaw extraction assumes the ArUco marker's X-axis is aligned with the world X-axis (i.e., the marker is placed in a specific orientation). If the marker is placed rotated (e.g., 45°), there will be a constant yaw offset in the position estimate. The fix is to always place markers consistently, or add a `marker_yaw_deg` field to the marker map YAML.

---

## Update — 2026-05-21 (Opus 4.7 pre-flight review)

A second review before the first field test added two safety features and surfaced one
critical configuration requirement.

### A. Vision-loss failsafe → LAND

**File:** `flight_controller_node.py` (`control_loop`, `aruco_callback`, `state_callback`)

Previously, when the marker left the camera FOV, `position_estimator_node` kept
re-publishing the *last* pose. The PID then held against a frozen setpoint while the
drone physically drifted away blind — the controller literally could not see the error.

Now the flight controller tracks the timestamp of the last ArUco detection. While armed
and in GUIDED, if no detection arrives for `vision_loss_timeout` seconds (default 2.0),
it commands **LAND**. The trigger is one-shot and re-arms on the next fresh detection.

```yaml
vision_loss_timeout: 2.0   # seconds; lower = lands sooner
```

### B. GPS-denied arming (no GPS module)

**File:** `flight_controller_node.py` (`_check_mavros_connection`, `_publish_ekf_origin`)

The Pixhawk refuses to arm in position-controlled modes without a position source. With
no GPS module, the EKF needs its **origin set explicitly** — this is the piece that is
easy to miss. On MAVROS connection, the node now publishes a fixed origin to
`/mavros/global_position/set_gp_origin` a few times (the absolute lat/lon is arbitrary
for local vision navigation).

```yaml
set_ekf_origin: true
ekf_origin_lat: 0.0
ekf_origin_lon: 0.0
ekf_origin_alt: 0.0
```

> Setting the origin from code is **necessary but not sufficient.** It only works
> together with the ArduPilot parameters below. Do **not** brute-force `ARMING_CHECK=0`
> — arming in GUIDED still needs a healthy EKF position, which only the vision source
> provides.

#### Required ArduPilot params for GPS-denied vision flight (Mission Planner)

| Parameter | Value | Reason |
|---|---|---|
| `GPS_TYPE` | `0` | No GPS hardware — stop the FCU looking for it |
| `AHRS_EKF_TYPE` | `3` | Use EKF3 |
| `EK3_ENABLE` | `1` | Enable EKF3 |
| `EK3_SRC1_POSXY` | `6` | Horizontal position source = ExternalNav (vision) |
| `EK3_SRC1_VELXY` | `6` | Horizontal velocity source = ExternalNav |
| `EK3_SRC1_POSZ` | `1` | Altitude source = Baro (`6` for vision Z if preferred) |
| `EK3_SRC1_YAW` | `6` | **Heading from ExternalNav** — see critical note below |
| `VISO_TYPE` | `1` | Accept MAVLink vision position |
| `ARMING_CHECK` | keep enabled | Relax only the GPS bit if needed — never set to 0 |

### C. CRITICAL — heading frame must match the position frame

This is the most likely remaining cause of residual drift. The vision position X/Y axes
are the **marker's printed axes**. If ArduPilot takes yaw from the **compass** (north)
while position comes from the marker frame, the two disagree by the marker's yaw offset
and the drone drifts/circles. Resolve it **one** of two ways:

1. Physically align the marker's X-axis with the drone's takeoff/forward heading, **or**
2. Set `EK3_SRC1_YAW = 6` (ExternalNav) so heading and position share the marker frame.

### Pre-flight bench check (no props) for the two new features

- Power on, connect MAVROS → confirm `Published EKF origin ...` appears in the log and
  the Pixhawk pre-arm GPS complaint clears (arming becomes possible).
- Arm, switch to GUIDED with the marker visible, then cover the camera → confirm the
  node logs `VISION LOST ... commanding LAND` after ~2 s and the mode flips to LAND.
