# Flight Test Logging

Use the flight-test recorder whenever running bench tests or low-altitude flight
tests. It writes compact logs on the drone/Pi that can be reviewed later by the
team or an AI agent.

## Start A Recorded Test

After rebuilding and sourcing the workspace on the Pi:

```bash
ros2 launch warehouse_drone_nav warehouse_nav.launch.py record_test:=true recorder_session:=basement_loiter_01
```

If `recorder_session` is empty, the recorder creates a timestamped session name.

Default output directory:

```text
~/drone_test_logs/<session_name>/
```

## Files To Send Back

Send the whole session folder, especially:

```text
summary.md
params.md
notes.md
samples.csv
```

Fill in `notes.md` immediately after the test while the details are fresh:

- location and lighting
- marker ID/size/orientation
- battery and payload
- mode sequence
- drift direction
- whether the marker visibly left the camera view

## What The Recorder Captures

`samples.csv` records:

- MAVROS mode, armed state, connection state
- `/drone/local_position`
- `/mavros/vision_pose/pose`
- `/drone/odometry`
- `/mavros/setpoint_raw/local` XY velocity and Z setpoint
- `/aruco/poses`
- `/aruco/ids`
- image brightness and blur metrics from `/camera/image_raw`

`params.md` records selected ArduPilot/MAVROS params through
`/mavros/param/get`:

- `EK3_SRC1_*`
- `VISO_TYPE`
- `GPS_TYPE`
- `PSC_*`
- `MOT_THST_HOVER`
- `MOT_HOVER_LEARN`

If the MAVROS parameter service is unavailable, `params.md` will say so; capture
the same parameters manually from Mission Planner.

## How To Interpret Camera Metrics

Dark basement/parking tests are risky for webcam-based ArUco tracking. Check:

- `brightness_mean`: low values mean the image is dark.
- `brightness_std`: very low values mean poor contrast.
- `blur_laplacian_var`: low values mean the frame is blurry or defocused.
- `dark_pixel_pct`: high values mean much of the image is nearly black.
- `aruco_age_sec`: rising values mean the marker has not been detected recently.
- `aruco_count`: should stay above zero while the marker is in view.

If drift starts when brightness drops, blur drops, or `aruco_age_sec` rises, the
camera feed is likely contributing to the instability.

## Practical Lighting Fixes

- Add diffuse light aimed at the marker, not directly into the camera.
- Avoid glossy acrylic glare over the marker.
- Use a larger marker or higher-contrast print if the camera is noisy.
- Lock camera exposure/focus if the webcam supports it.
- Keep shutter speed high enough to avoid motion blur during yaw and takeoff.
