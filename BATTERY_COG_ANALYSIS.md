# Battery Placement & Center of Gravity Analysis

**Date:** 2026-05-21  
**Context:** F450 frame build with Pixhawk 6C, Raspberry Pi 4B, downward ArUco camera

---

## Problem

The battery is currently held to the drone body by velcro strap only. This means the battery position shifts slightly every time the battery is swapped. A shifted battery moves the system's Center of Gravity (CoG) away from the frame center, causing a directional drift force during hover that requires constant manual correction.

---

## Why Battery Position Affects Drift

A quadcopter's flight controller assumes the CoG sits directly below the geometric center of the frame (where the FC is mounted). When the battery shifts off-center, the drone must lean permanently in the opposite direction to maintain level flight:

```
Ideal layout:             Battery shifted forward:

   M1      M2                M1      M2
    \      /                  \      /
     [  FC  ]                  [ FC  ]---[BATTERY]
    /      \                  /      \
   M3      M4                M3      M4

CoG = frame center        CoG = forward of FC
Hover: all motors equal   Hover: front motors run harder
                          Drone leans back to compensate
```

A permanently leaned drone means gravity has a constant horizontal component pointing in the direction of the CoG offset. This produces a **constant, directional drift force** — distinct from random noise drift. It always pushes toward the same side for a given battery position, and it never self-corrects unless the battery is repositioned.

---

## Software Mitigation (Partial)

### What helps: the integral term (Ki) in the Python PID

The flight controller (`flight_controller_node.py`) now includes a Python-side XY velocity PID (see `IMPROVEMENTS.md`). A constant CoG offset produces a constant drift force, which is exactly the scenario the integral term is designed to handle:

- The integral accumulates over time until it produces a counterforce equal to the drift force
- Typical convergence time: **10–30 seconds** after entering GUIDED mode
- During convergence the drone is still drifting — the marker can leave the camera FOV

**Recommended Ki value if battery placement varies:** `pid_ki: 0.08` (up from the default 0.05) in `src/config/flight_controller.yaml`. This builds up the correction force faster.

### Limitations of software compensation

| Limitation | Why it matters |
|---|---|
| Integral resets on every mode switch | Re-learns the offset from scratch each GUIDED entry |
| 10–30s convergence window | Drone drifts during this period; marker may leave FOV |
| CoG lean tilts the camera | Non-level camera introduces small systematic error in ArUco position estimate that software cannot fully separate from real position error |
| Does not fix the root cause | Battery will continue to shift unpredictably |

### What does not help: ArduPilot TRIM parameters

ArduPilot has `TRIM_ROLL` and `TRIM_PITCH` parameters that add a constant attitude offset to compensate for CoG offset. These would need to be manually re-tuned after every battery swap — not practical for regular operation.

---

## Hardware Fix: 3D Print a Battery Mount (Recommended)

A fixed battery mount eliminates the problem entirely. Software compensation becomes a safety net rather than a primary correction mechanism.

### Design requirements

1. **Hard-stop registration** — the battery must seat in the same position every time, within ±1 mm tolerance. Use a slot or rail with a defined end-stop rather than friction or velcro alone.

2. **Secure retention** — a latch or strap that pulls the battery firmly against the end-stop. Velcro alone allows creep under vibration.

3. **Correct CoG position** — the slot must be located such that the combined CoG of frame + battery falls at or very near the FC mounting point.

### How to calculate the correct battery position

**Step 1 — Find the empty frame CoG**  
Hang the empty frame (no battery) from a string attached to the center bolt. Mark where it naturally balances along both axes.

**Step 2 — Calculate required battery CoG position**

```
CoG_combined = (M_frame × CoG_frame + M_battery × CoG_battery) / (M_frame + M_battery)
```

Set `CoG_combined = FC_position` and solve for `CoG_battery`:

```
CoG_battery = (CoG_FC × (M_frame + M_battery) - M_frame × CoG_frame) / M_battery
```

**Example values for a typical F450 build:**
- Frame mass: ~300 g
- Battery (3S 2200mAh): ~180 g
- The battery usually needs to sit roughly centered under the FC, offset slightly toward the heavier side of the frame (typically where the FC + ESC stack concentrates mass)

**Step 3 — Verify after printing**  
Hold the assembled drone level by the center bolt. Confirm it does not tip more than ~5° in any direction. Adjust the mount position if needed before finalizing the print.

### Vibration note

Velcro is a moderate vibration absorber. A hard-mounted battery transfers more vibration to the frame and into the IMU. Add a thin layer of foam or anti-vibration tape between the battery and the printed case to preserve IMU signal quality.

---

## Recommended Action Plan

| Priority | Action | Expected outcome |
|---|---|---|
| Immediate | Set `pid_ki: 0.08` in `flight_controller.yaml` | Faster integral convergence; reduces drift window to ~15s |
| Short term | Add battery position marks on the frame so velcro placement is repeatable | Reduces variance between swaps |
| Long term | 3D print battery mount with hard-stop registration | Eliminates CoG shift entirely; makes Ki compensation reliable and fast |

---

## Pre-Flight Checklist Addition

Add this step to the pre-flight procedure after every battery swap:

- [ ] Hold the drone level by the center bolt (or place on a flat surface)
- [ ] Confirm the frame does not tip more than ~5° in roll or pitch
- [ ] If tipping is observed, reposition the battery before flight
