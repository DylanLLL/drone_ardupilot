# GoPro HERO4 as the Main Video Source

The GoPro HERO4 replaces the Logitech C270 as the default camera for ArUco
detection. This document covers the required hardware chain, GoPro settings,
the **mandatory recalibration**, and — importantly — what this swap does and
does not fix about vibration.

---

## Read this first: what the GoPro does and doesn't fix

**The HERO4 has no image stabilization.** GoPro introduced electronic
stabilization (EIS) with the HERO5 and HyperSmooth with the HERO7 — the HERO4
records exactly what the shaking sensor sees, same as the C270.

**That is actually fine, because EIS would be harmful here.** EIS works by
dynamically cropping and warping the image. ArUco pose estimation converts
pixel coordinates into metric positions through a fixed calibration matrix;
a stabilizer that shifts the crop frame-to-frame silently invalidates that
matrix, injecting position error straight into the EKF. For fiducial-based
localization you want a *rigid, calibrated, unstabilized* image. If you ever
use a camera that has EIS, turn it off.

**What the GoPro genuinely improves:**
- **~2× angular resolution** at 1280×720 vs the C270's 640×480 — more pixels
  per marker means detection survives more blur and works from higher altitude.
- **A far better sensor and lens** — better low-light behavior, so the shutter
  stays faster in warehouse lighting (less motion blur per frame).
- **A wider field of view** — the marker stays in frame through larger
  tilt/drift excursions, which directly addresses losing sight of the tag.

**What actually fixes vibration** (do these regardless of camera):
1. **Mechanically isolate the camera mount** — rubber vibration dampers /
   gel pads between frame and camera. This is the single highest-impact fix.
2. **Balance the propellers** — the dominant vibration source on an F450.
3. **Fast shutter** — if your HERO4 firmware exposes Protune shutter control
   in video mode, set 1/240 or faster. Shorter exposure = less smear per frame.
4. Both cameras are rolling-shutter; under heavy vibration expect "jello"
   distortion, which damping (not a different camera) removes.

---

## Hardware chain

The HERO4 **cannot act as a USB webcam** (UVC webcam mode arrived with the
HERO8), and its WiFi preview stream has 1–2 s latency — more than the 2 s
vision-loss failsafe, i.e. unflyable. The only flight-viable path is HDMI:

```
GoPro HERO4 ──micro-HDMI──> HDMI-to-USB capture card (UVC) ──USB──> RPi 4B
```

**You need:**
- Micro-HDMI (type D) → HDMI cable (short; buy or make a light one)
- HDMI-to-USB capture card that is **UVC class** (driverless). The common
  ~$10–20 MacroSilicon MS2109/MS2130 "HDMI USB video capture" dongles work
  and appear as a normal `/dev/videoN` device.
- USB power lead for the GoPro — HDMI-out drains the battery quickly; the
  HERO4 runs fine on USB power with the battery inserted.

Weight note: GoPro (~85 g) + capture card + cables is noticeably heavier than
the C270 (~75 g with case removed, less stripped). Mount it as close to the
drone's center of gravity as possible and re-check hover throttle.

---

## GoPro settings

On the camera (Settings / mode menus):

| Setting | Value | Why |
|---|---|---|
| Mode | Video, 720p60 (or 1080p30) | HDMI live-out follows the video mode |
| FOV | **Medium** (recommended) or Narrow | Wide has extreme barrel distortion; Medium balances keeping the marker in view vs distortion. Narrow ≈ C270's FOV, least distortion |
| Protune | On, shutter 1/240+ (if your firmware has video shutter) | Less motion blur |
| OSD (on-screen display) | **Off** | Overlays would be baked into the captured frames and confuse detection |
| Auto Off | **Never** | The camera sleeping mid-flight = vision loss → forced LAND |
| QuikCapture | Off | Prevents accidental mode changes |

The capture card scales the HDMI feed to whatever resolution ROS requests —
the ROS-side resolution is set in `src/config/camera_gopro_hero4.yaml`
(default 1280×720 @ 30 fps, MJPEG).

**The FOV setting changes the lens geometry. Calibrate at the FOV you fly
with, and never change it afterwards without recalibrating.**

---

## Verify the capture card

Plug everything in, put the GoPro in video mode (HDMI live preview active):

```bash
# Find the device
v4l2-ctl --list-devices

# Confirm it offers MJPG at 1280x720@30
v4l2-ctl -d /dev/video0 --list-formats-ext
```

If the Pi enumerates more than one video device (or the number changes across
boots), use the stable path and put it in `camera_gopro_hero4.yaml`:

```bash
ls -l /dev/v4l/by-id/
# e.g. video_device: '/dev/v4l/by-id/usb-MACROSILICON_USB_Video-...-video-index0'
```

---

## Calibration (REQUIRED — the system refuses to run without it)

The GoPro profile ships with a **placeholder** calibration
(`src/config/camera_calibration_gopro_hero4.yaml`, all zeros).
`aruco_detector_node` rejects a zero focal length and logs an error until a
real calibration is in place — this is deliberate, so an uncalibrated setup
fails on the bench, not in the air.

1. Set the GoPro to the exact mode + FOV you will fly with.
2. Start the camera:
   ```bash
   ros2 run usb_cam usb_cam_node_exe --ros-args \
       --params-file $(ros2 pkg prefix warehouse_drone_nav)/share/warehouse_drone_nav/config/camera_gopro_hero4.yaml \
       -r __ns:=/camera
   ```
3. Run the calibrator (8×6 checkerboard, 24 mm squares — same board as before):
   ```bash
   ros2 run camera_calibration cameracalibrator \
       --size 8x6 --square 0.024 \
       image:=/camera/image_raw camera:=/camera
   ```
4. Move the board through the **entire frame, especially corners and edges**
   — that's where the GoPro's barrel distortion is strongest and where the
   plumb_bob model needs data. Also vary distance and tilt until the
   X/Y/Size/Skew bars are green.
5. Click CALIBRATE, then SAVE. Extract `ost.yaml` from the saved tarball and
   copy its matrices into `src/config/camera_calibration_gopro_hero4.yaml`,
   keeping `camera_name: gopro_hero4`.
6. Rebuild so the installed copy updates:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select warehouse_drone_nav && source install/setup.bash
   ```

**Bench-verify the scale before flying** (Testing Guide, Test 1): hold the
drone at a tape-measured distance from the marker and confirm the logged
distance matches. With a good calibration `distance_scale_factor` stays `1.0`.

---

## Running with each camera

The camera is now launched from profiles — no hardcoded camera settings
anywhere else.

```bash
# Full system, GoPro (default)
ros2 launch warehouse_drone_nav warehouse_nav.launch.py

# Full system, fall back to the C270
ros2 launch warehouse_drone_nav warehouse_nav.launch.py camera:=c270

# Don't launch a camera (e.g. you start it by hand in another terminal)
ros2 launch warehouse_drone_nav warehouse_nav.launch.py camera:=none

# Detection-only bench test works the same way
ros2 launch warehouse_drone_nav aruco_test.launch.py [camera:=c270|none]

# tmux startup script
./start_drone.sh                      # GoPro (default)
CAMERA_PROFILE=c270 ./start_drone.sh  # C270 fallback
```

Everything downstream (`/camera/image_raw`, `/camera/camera_info`, the
detector, estimator, flight controller, recorder) is unchanged — the swap is
entirely contained in the camera profile.

---

## Pre-flight checks specific to this setup

- **CPU headroom:** 720p MJPEG decode + detection costs more than the C270's
  640×480. On the bench run `top` — if `aruco_detector_node` or `usb_cam`
  saturate a core, or the detection log rate visibly drops, set the profile to
  640×480 and **recalibrate at that resolution**.
- **Latency sanity:** wave a hand over the marker while watching
  `/aruco/detection_image` — the HDMI + capture chain adds ~50–150 ms on top
  of the camera. If position hold oscillates where the C270 didn't, treat it
  like the EMA-lag problem in IMPROVEMENTS.md: keep `position_filter_alpha`
  at 0.6–0.8 and don't add more smoothing.
- **Vision-loss failsafe still works end-to-end:** with props off, cover the
  lens / unplug HDMI and confirm `VISION LOST ... commanding LAND` after ~2 s.
  This is also your protection if the GoPro overheats or sleeps mid-flight.
- **GoPro on USB power, Auto Off = Never**, battery inserted, lens cover off.

## Troubleshooting

- **No `/dev/videoN`:** capture card not UVC or bad cable — test the card
  with a laptop first; check `dmesg | tail` after plugging.
- **Black frames:** GoPro not in live preview (press mode button), or HDMI
  cable seated poorly.
- **"camera_info has zero focal length" error:** you're still on the
  placeholder calibration — complete the calibration section above.
- **Only low fps at 720p:** you're in YUYV — `pixel_format` must be
  `mjpeg2rgb` (USB2 bandwidth limit).
- **Detection worse at frame edges:** normal for a wide lens; keep the marker
  away from extreme corners or switch FOV to Narrow (then recalibrate).
