# ArUco Marker Orientation Guide

## Why Marker Orientation Matters

ArUco markers have a **specific orientation** that defines their coordinate frame. If placed incorrectly on the ground, the drone will:
- Move in wrong directions (e.g., left when commanded forward)
- Rotate unexpectedly when entering GUIDED mode
- Report incorrect X/Y offsets

## Understanding ArUco Marker Structure

### Marker Anatomy:
```
        TOP EDGE (this has meaning!)
     ┌─────────────────┐
     │ ■■■■■■■■■■■■■■■ │  ← Black border
     │ ■ □ ■ □ ■ ■ □ ■ │
     │ ■ ■ □ ■ □ ■ ■ ■ │  ← Unique ID pattern
     │ ■ □ ■ ■ ■ □ ■ ■ │     (NOT symmetric!)
     │ ■ ■ ■ □ ■ ■ □ ■ │
     │ ■■■■■■■■■■■■■■■ │
     └─────────────────┘
       BOTTOM EDGE
```

**Key Point**: The pattern inside is **NOT symmetric** - rotating the marker changes its detected orientation!

## Correct Ground Placement

### For Downward-Facing Camera Setup:

```
        WORLD COORDINATE FRAME
              NORTH (X+)
                  ↑
                  │
     WEST (Y-) ←──┼──→ EAST (Y+)
                  │
                  ↓
              SOUTH (X-)


        Marker on Ground:
     ┌─────────────────┐
     │   ▲ RED AXIS    │  ← This edge points NORTH (X+ / Forward)
     │   │ (X+)        │
     │   │             │
     │ ←─┼─→ GREEN     │  ← Green axis points EAST (Y+ / Right)
     │   │   (Y+)      │
     │   │             │
     │   ↓ BLUE (Z+)   │  ← Blue points UP (away from ground)
     └─────────────────┘
```

### Rule of Thumb:
**The marker's RED axis (top edge) should point in your FORWARD direction (North / X+)**

If your warehouse has a natural "forward" direction (e.g., along aisles), align the red axis with that direction.

## How to Verify Marker Orientation

### Method 1: Visual Feedback (Recommended)

The system now displays a **red arrow** on the camera feed showing marker orientation:

1. Launch the system:
   ```bash
   ros2 launch warehouse_drone_nav warehouse_nav.launch.py
   ```

2. View the camera feed with ArUco overlay:
   ```bash
   ros2 run rqt_image_view rqt_image_view /aruco/detection_image
   ```

3. **What you'll see**:
   ```
   Camera feed with overlay:
   - Green square outline around marker
   - 3D axes drawn on marker:
     * RED axis = marker's X+ (forward)
     * GREEN axis = marker's Y+ (right)
     * BLUE axis = marker's Z+ (up from marker)
   - Red arrow pointing in marker's forward direction
   - Text label: "ID:0 D:1.50m Yaw:0deg"
   ```

4. **Correct orientation check**:
   - Stand behind the marker (south side)
   - Look at camera feed
   - **RED arrow should point AWAY from you** (toward north)
   - **Yaw angle should be close to 0°** (±10° is acceptable)

### Method 2: Terminal Logs

Watch the terminal output:
```bash
[aruco_detector]: Detected marker ID 0 at distance 1.50m | Yaw=2.5°
```

**Yaw angle interpretation**:
- **0°**: Marker aligned perfectly (red axis points north)
- **90°**: Marker rotated 90° clockwise (red axis points east)
- **180° or -180°**: Marker upside down (red axis points south)
- **-90°**: Marker rotated 90° counter-clockwise (red axis points west)

**Target**: Yaw should be close to **0°** (within ±10°)

### Method 3: Drone Behavior Test

1. Place drone ~50cm above marker (hovering in ALT_HOLD)
2. Position drone 0.5m NORTH of marker (in front)
3. Switch to GUIDED mode
4. **Expected**: Drone moves SOUTH (backward) to center over marker
5. **If wrong**: Drone moves in unexpected direction → marker is rotated incorrectly

## Common Mistakes & Fixes

### Problem 1: Drone Rotates 90° When Entering GUIDED
**Cause**: Marker is rotated 90° from correct orientation

**Fix**: Rotate marker 90° so red arrow points north

### Problem 2: Drone Moves Left When Commanded Forward
**Cause**: Marker coordinate frame doesn't match world frame

**Fix**:
1. Check yaw angle in logs
2. Rotate marker to get yaw ≈ 0°

### Problem 3: Drone Moves in Opposite Direction
**Cause**: Marker is upside down (180° rotation)

**Fix**: Rotate marker 180° so red arrow points north (not south)

## Multiple Markers Setup

When placing multiple markers (as in marker_map_sample.yaml):

```yaml
markers:
  - id: 0
    position: {x: 0.0, y: 0.0, z: 0.0}  # Origin
  - id: 1
    position: {x: 2.0, y: 0.0, z: 0.0}  # 2m north of origin
  - id: 2
    position: {x: 4.0, y: 0.0, z: 0.0}  # 4m north of origin
```

**All markers must have the SAME orientation**:
- All red axes point north (X+)
- All green axes point east (Y+)
- All blue axes point up (Z+)

### Verification:
1. View all markers in camera feed
2. **All red arrows should point in same direction**
3. **All yaw angles should be similar** (within ±5°)

## Using OpenCV Axes Visualization

The 3D axes drawn on the marker show:

```
         RED
          ↑
          │
GREEN ←───┼ (marker center)
          │
          ↓
         BLUE (into ground when on floor)
```

**For ground markers with downward camera**:
- RED axis: Points along marker's top edge (should point north)
- GREEN axis: Points along marker's right edge (should point east)
- BLUE axis: Points UP from marker (perpendicular to ground)

## Quick Setup Checklist

- [ ] Print ArUco markers at correct size (15cm outer border)
- [ ] Laminate markers to prevent damage
- [ ] Tape markers flat on ground (no wrinkles or curves)
- [ ] Align all markers with red axis pointing north (forward direction)
- [ ] Verify orientation with camera feed (red arrow points north)
- [ ] Check yaw angle in logs (should be ≈ 0°)
- [ ] Test with drone: switching to GUIDED should center smoothly
- [ ] All markers in map have consistent orientation

## Troubleshooting

### Issue: "Yaw angle keeps changing"
- Marker is not flat on ground
- Marker is wrinkled or damaged
- Lighting is poor (causing detection instability)

### Issue: "Red arrow points in random directions"
- Multiple markers with different orientations
- Check each marker individually
- Re-align all markers to same direction

### Issue: "No red arrow visible in camera feed"
- Marker not detected
- Check distance (should be 0.3m - 3m for 15cm markers)
- Check lighting (avoid shadows and glare)
- Verify camera calibration

## Camera Feed Visualization

To view the ArUco detection overlay:

```bash
# Method 1: rqt_image_view (GUI)
ros2 run rqt_image_view rqt_image_view

# Then select topic: /aruco/detection_image

# Method 2: Check if images are being published
ros2 topic hz /aruco/detection_image
```

You should see:
- Green marker outlines
- 3D RGB axes on each marker
- Red orientation arrow
- Text showing ID, distance, and yaw angle

## Summary

✅ **Correct Setup**:
- Red axis (top edge) → Points NORTH (forward)
- Green axis (right edge) → Points EAST (right)
- Yaw angle ≈ 0° in logs
- Red arrow points forward in camera feed
- Drone centers smoothly over marker in GUIDED mode

❌ **Incorrect Setup**:
- Yaw angle is 90°, 180°, or -90°
- Red arrow points wrong direction
- Drone rotates or moves unexpectedly in GUIDED mode
- Different markers have different yaw angles

**When in doubt**: Check the camera visualization - the red arrow shows exactly where the marker's "forward" direction is pointing!
