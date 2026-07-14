# LOITER Drift Diagnostic Proposal

## Context

During the latest LOITER test, the drone armed on the clear acrylic platform,
lifted off smoothly, held for a few seconds, then drifted steadily toward
forward-right until the takeoff-pad marker left the camera field of view. The
vision-loss failsafe then commanded LAND and the drone landed safely.

The team's parameter review confirms the highest-risk issue:

- `EK3_SRC1_YAW = 1` means ArduPilot is using compass yaw.
- Vision position is expressed in the marker/world frame.
- The last session showed marker yaw near `-179.9 deg`.

That means the fused heading and fused position are likely separated by about
180 degrees. In LOITER, ArduPilot then applies XY corrections in the wrong frame,
which matches the observed smooth, constant diagonal drift.

## Current Code State

The current branch is `claude/hopeful-cori-yOuAQ`, with remote head `f910338`.
That commit already includes the main code/config cleanup requested by the team:

- `marker_size: 0.15`
- `marker_size_ids: [-1]`
- `marker_size_values: [0.0]`
- `distance_scale_factor: 1.0`
- `marker_map_takeoff.yaml` describes ID 0 as a 15 cm takeoff marker.
- `position_estimator.yaml` exposes:
  - `camera_offset_forward`
  - `camera_offset_right`
  - `camera_offset_down`

Local follow-up adds a backward-compatible marker-ID side channel:

- `aruco_detector_node.py` publishes detected IDs on `/aruco/ids`.
- `position_estimator_node.py` uses `/aruco/ids` to choose the matching marker
  from the marker map instead of assuming the first configured marker.

## Primary Diagnosis

### 1. Heading-Frame Mismatch

Likelihood: confirmed high.

Current issue:

- Position comes from the ArUco marker frame.
- Yaw comes from the compass when `EK3_SRC1_YAW=1`.
- The observed marker yaw near `-179.9 deg` indicates the marker frame is
  reversed relative to the expected aircraft/world heading.

Expected symptom:

- Smooth drift in a consistent direction.
- Not necessarily oscillation.
- Corrections look confident but are applied in the wrong XY frame.

Required action before the next flight:

- Bench-confirm marker/drone yaw alignment.
- Set `EK3_SRC1_YAW=6` only after vision yaw is verified to have the correct sign
  and zero reference.

## Secondary Contributors

### 2. Dark / Blurry Camera Input

Likelihood: plausible after the basement-parking test.

ArUco tracking depends on clear marker corners. In a dark area, a webcam can
increase exposure time and gain, which creates motion blur and noisy edges. That
can cause intermittent marker detection, pose jitter, or delayed corrections.
Even if the EKF/heading settings are correct, noisy vision can make LOITER chase
bad pose updates or coast after detections drop out.

Action:

- Run the next test with the flight-test recorder enabled.
- Watch `brightness_mean`, `brightness_std`, `blur_laplacian_var`,
  `aruco_count`, and `aruco_age_sec` in `samples.csv`.
- Add diffuse lighting and avoid glare on the acrylic/marker.
- Lock camera exposure/focus if the webcam supports it.

### 3. Camera Lever Arm

Likelihood: real contributor, secondary to heading-frame mismatch.

The estimator's raw pose math returns the camera's world position. If the camera
is mounted forward of the FC/body center, a constant heading creates mostly a
fixed offset. That fixed offset is not usually destabilizing by itself.

When the drone yaws, however, the forward camera offset rotates around the body
center. Without lever-arm compensation, the EKF sees false XY movement and may
chase it. This can compound the heading-frame mismatch.

Action:

- Measure FC/body-center to lens offset.
- Set `camera_offset_forward` in meters, for example `0.08` for 8 cm forward.
- Use `camera_offset_right` if the lens is right/left of center.
- Leave `camera_offset_down` at `0.0` unless Z offset is needed later.

### 4. Marker Size / Distance Scale

Likelihood: reduced after `f910338`, still needs bench verification.

Current config assumes one 15 cm marker:

```yaml
marker_size: 0.15
marker_size_ids: [-1]
marker_size_values: [0.0]
distance_scale_factor: 1.0
```

The previous `distance_scale_factor: 0.632` was too large to be a normal lens
calibration correction. It was likely compensating for a wrong assumed marker
size. With a true 15 cm marker, re-baseline at `1.0`.

### 5. ArduPilot PSC Gains

Likelihood: tuning risk, not the primary observed failure.

The team found GPS-like defaults:

- `PSC_POSXY_P = 1`
- `PSC_VELXY_P = 2`

These are aggressive for 30 Hz vision. Aggressive gains normally show up as
oscillation, which was not the main symptom, but softening is still sensible
before the next flight.

Recommended starting values:

- `PSC_POSXY_P = 0.3`
- `PSC_VELXY_P = 0.8`

### 6. Hover Throttle / Payload State

Likelihood: setup risk.

The reported `MOT_THST_HOVER` change from `0.34` to `0.61` is large. Reconfirm
payload/battery state and keep hover learning enabled:

- `MOT_HOVER_LEARN = 2`

## Mission Planner Changes

Do these only after the no-prop yaw tests confirm vision yaw sign and alignment.

```text
EK3_SRC1_YAW   1 -> 6
PSC_POSXY_P    1 -> 0.3
PSC_VELXY_P    2 -> 0.8
MOT_HOVER_LEARN     2
```

Rationale:

- `EK3_SRC1_YAW=6` makes heading and position share the ExternalNav/marker frame.
- Softer PSC gains reduce overreaction to vision noise and latency.
- Hover learning adapts to real takeoff weight.

## Bench Tests Before Flight

Run all tests with props removed.

Enable the recorder during bench and flight tests:

```bash
ros2 launch warehouse_drone_nav warehouse_nav.launch.py record_test:=true recorder_session:=basement_loiter_01
```

Logs are written under `~/drone_test_logs/<session_name>/`. See
`FLIGHT_TEST_LOGGING.md` for the files to send back.

### Test 1: Marker Size / Distance Scale

Goal: verify that the 15 cm marker and `distance_scale_factor=1.0` produce real
distances.

Steps:

1. Measure the physical marker's outer black-border size.
2. Confirm it is ID 0 and 15 cm.
3. Start the vision stack.
4. Hold the camera at a measured height over the marker.
5. Compare logged corrected distance to the measured physical distance.

Pass condition:

- Logged corrected distance is close to the measured distance.

If failed:

- If the error is a constant ratio, set:

```text
distance_scale_factor = real_distance / reported_distance
```

### Test 2: Yaw Alignment

Goal: prove that marker-frame yaw matches aircraft heading before using
ExternalNav yaw.

Steps:

1. Align the drone nose with the marker red X axis.
2. Start the vision stack.
3. Watch `Yaw_raw` from `position_estimator_node`.

Pass condition:

- Nose aligned to marker red X gives yaw near `0 deg`.

Fail condition:

- Yaw is near `+180 deg` or `-180 deg`.

If failed:

- Rotate the marker 180 degrees and repeat.
- Do not fly LOITER with `EK3_SRC1_YAW=6` until this test passes.

### Test 3: Translation Axis Signs

Goal: confirm position X/Y signs are consistent with the intended world frame.

Steps:

1. Echo `/drone/local_position`.
2. Keep yaw fixed.
3. Move the drone 10 cm right, then back to center.
4. Move the drone 10 cm forward, then back to center.

Pass condition:

- The same axis/sign changes consistently each time.
- Values return close to origin when returned to center.

If failed:

- Re-check marker orientation and `_R_BODY_CAM` mount assumptions.

### Test 4: Pure Yaw / Lever Arm

Goal: tune camera offset so pure yaw does not look like translation.

Steps:

1. Put the drone over the marker.
2. Set measured `camera_offset_forward/right/down`.
3. Rotate the drone in place by 90 degrees.
4. Rotate in place by 180 degrees.
5. Watch `/drone/local_position`.

Pass condition:

- X/Y position stays nearly constant during pure yaw.

If failed:

- Tune `camera_offset_forward` and `camera_offset_right`.
- If signs look swapped or reversed, revisit camera mount orientation.

### Test 5: Parameter Capture

Record these Mission Planner parameters before and after edits:

```text
EK3_SRC1_POSXY
EK3_SRC1_POSZ
EK3_SRC1_VELXY
EK3_SRC1_YAW
VISO_TYPE
GPS_TYPE
PSC_POSXY_P
PSC_VELXY_P
PSC_VELXY_I
PSC_VELXY_D
PSC_VELXY_FILT_HZ
MOT_THST_HOVER
MOT_HOVER_LEARN
```

## Recommended Sequence

1. Rebuild/deploy the current branch on the Pi.
2. Start every test with `record_test:=true`.
3. Run Test 1 to confirm 15 cm marker scale.
4. Run Test 2 and physically fix marker orientation until yaw reads near `0 deg`.
5. Run Test 3 to verify translation signs.
6. Measure/set camera lever arm and run Test 4.
7. In Mission Planner, set:
   - `EK3_SRC1_YAW=6`
   - `PSC_POSXY_P=0.3`
   - `PSC_VELXY_P=0.8`
   - `MOT_HOVER_LEARN=2`
8. Re-test LOITER low and slow with immediate manual takeover available.

## Current Recommendation

Do not add more drift-control logic before the no-prop checks. The strongest fix
is not more Python control code; it is making ArduPilot fuse position and yaw in
the same frame, then reducing LOITER gains for vision.
