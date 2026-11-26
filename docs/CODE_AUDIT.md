# Code and Architecture Audit of Version 1

This is an honest review of the first version of the project (`camera_node.py`, `precision_landing.py`, the README and the PDF guide). It lists what is good, what is risky, and what was changed in version 2.

**Scope note:** `config/params.yaml` was not included in the files that were reviewed, so its contents are not covered. What matters is how the code uses it, and the code does not read it at all (see section 6).

**Testing note:** The v2 control logic was checked with a simplified simulation of a drone (no PX4, no Gazebo): a normal landing, a short marker loss, a long marker loss, and a start on the ground. This catches logic mistakes but does **not** replace testing in PX4 SITL. Real flight behaviour (wind, delays, camera mounting) can only be confirmed there.

---

## 1. What is solid

* **Perception and control are separate.** Two small programs with a clear message between them is the right design and is easy to explain and test.
* **The state machine is simple and readable.** One state per behaviour, with clear exit conditions.
* **PX4 does the dangerous low-level work.** Stabilisation, RTL and the final touchdown stay with PX4.
* **The 3-second "marker must be stable" gate** before taking control avoids reacting to a single false detection.
* **OFFBOARD pre-streaming** (sending setpoints before starting OFFBOARD) is correct and required by PX4.
* **A proportional controller on a normalised error** is a sensible starting point and works in a calm simulator.
* **Good documentation habits:** line-by-line comments, a troubleshooting table, and a clear launch order.

---

## 2. Weak or risky points (summary)

| # | Area | Severity | Problem in v1 | Fixed in v2 |
|---|---|---|---|---|
| 1 | Marker loss | **High** | Old `nx`/`ny` kept being used, so the drone kept flying on stale data | Hover, then climb slowly, then HOLD |
| 2 | Yaw control | **High** | Yaw rate had the wrong sign (turned away from the marker) and fought the sideways correction | Removed; heading held |
| 3 | Threads | **Medium** | Two threads shared several variables with no lock; one variable was reset by one thread and set by the other | Lock + one reading object |
| 4 | Telemetry | **Medium** | A new telemetry stream was opened on every 20 Hz loop | One background task keeps the latest value |
| 5 | Control gain | **Medium** | Same gain at every height, although the same pixel error means very different distances | Errors converted to metres; gain no longer depends on height |
| 6 | Alignment | **Medium** | One-way ALIGN to DESCEND; no way back if the drone drifted off | Descent slows with error; returns to ALIGN |
| 7 | Settings | **Medium** | `params.yaml` was documented but never loaded | Both nodes load it |
| 8 | Wrong marker | **Medium** | Always followed the first marker found, whatever its ID | `target_marker_id` |
| 9 | Crashes | **Medium** | If the MAVSDK thread crashed, the node kept running silently | Errors logged, ABORT to HOLD, clean exit |
| 10 | Camera node | **Low** | `imshow` and `print` every frame; every frame processed | Optional window, throttled logs, light debug output |

Details follow.

---

## 3. Threads and race conditions

**How v1 works.** `rclpy.spin()` runs the ROS 2 callbacks in the main thread. The MAVSDK loop runs in a second thread. Both used the same plain Python variables: `nx`, `ny`, `last_detection_time` and `first_detection_time`.

**Why it is a problem.**

1. **Two callbacks, two updates.** `nx` and `ny` arrived on separate topics, so the controller could read a new `nx` with an old `ny` (a mixed reading).
2. **A real race.** The ROS callback did "if `first_detection_time` is empty, set it". At the same moment the MAVSDK thread did "if marker lost, set it to empty". These two threads can overwrite each other, which can restart the 3-second timer or leave it set when it should not be.
3. **Wall clock time.** `time.time()` can jump when the computer's clock is adjusted. A timer should use `time.monotonic()`.
4. **Silent thread death.** An exception in the MAVSDK thread ends that thread only. The ROS side keeps spinning, so the program looks alive while nothing controls the drone.

**Why it mostly worked anyway.** Python's Global Interpreter Lock makes each simple assignment safe on its own. The problem is that several related values were updated *separately*, so the controller could see them half-updated.

**What v2 does.**

* One message (`/aruco_error`, `PointStamped`) carries both errors, so they always arrive together.
* The ROS callback creates one small read-only `Detection` object and swaps it in under a `threading.Lock`.
* Everything else (timers, filters, state) is touched **only** by the MAVSDK thread, so no other locking is needed.
* All timing uses `time.monotonic()`.
* The MAVSDK thread catches and logs exceptions, goes to HOLD, and signals the main thread to exit.

```python
# ROS callback (main thread)
det = Detection(msg.point.x, msg.point.y, time.monotonic())
with self._lock:
    self._latest = det

# Control loop (MAVSDK thread)
with self._lock:
    det = self._latest
```

---

## 4. Polling telemetry inside the 20 Hz loop

**v1:**
```python
async for pos in drone.telemetry.position_velocity_ned():
    self.current_z = pos.position.down_m
    break
```

**Problems.**

* Each pass opens a new subscription to the telemetry stream and throws it away after one value. That is wasteful at 20 times per second.
* The first value from a fresh stream is not guaranteed to be the newest one, so the altitude can be old without the code knowing.
* There was no check for "no altitude for a while".

**v2:** one background task runs for the whole flight and keeps the latest value (with a time stamp). The control loop just reads it. If the altitude is older than `telemetry_timeout_s`, the drone stops descending. The update rate is also requested (`telemetry_rate_hz`).

```python
async def _watch_position(self, drone):
    async for pv in drone.telemetry.position_velocity_ned():
        self.alt_m = -pv.position.down_m
        self.alt_time = time.monotonic()
```

**A point about the altitude itself.** `down_m` is measured from PX4's starting point, not from the ground. That is fine on flat ground in simulation. On a real drone use a rangefinder (distance sensor) or another proven height source.

---

## 5. Marker loss during ALIGN and DESCEND

**v1:** when the marker disappeared, nothing was published, so `nx` and `ny` stayed at their last values. The controller kept sending the same correction (and in DESCEND, kept going down). The "marker lost" information was computed (`detected_recent`) but only used inside the RTL state.

**Why it matters.** A single missed frame is harmless. A longer loss (glare, the marker leaving the view, the camera too close) means the drone flies blind and keeps descending. This is the most important safety gap in v1.

**v2 behaviour:**

| Situation | Response |
|---|---|
| Marker missing for up to 1 s | Hover (speed commands go to zero smoothly) |
| Missing 1 to 8 s | Climb slowly (up to a height limit) so the camera sees more ground |
| Marker seen again for 0.5 s | Back to ALIGN |
| Missing 8 s | ABORT: stop OFFBOARD, ask PX4 to HOLD |
| Below 1.5 m (FINAL_DESCENT) | Keep going straight down at slow speed. Very close to the ground the marker can fill or leave the view, so losing it is expected |

---

## 6. Constants versus `params.yaml`

The README says "all parameters are in `config/params.yaml`", and the PDF repeats numbers such as `Kp = 0.6`. In v1 the numbers were written directly in the Python files, and nothing read the YAML file. Editing the YAML changed nothing, which is confusing when tuning.

**v2:** both nodes declare **ROS 2 parameters** and read them from the file at start-up:

```bash
python3 src/precision_landing.py --ros-args --params-file config/params.yaml
```

The defaults in the code match the YAML file. If you start a node without the file, it still works with the defaults.

---

## 7. Control law

### 7.1 Fixed `KP = 0.6` on a normalised error

The error is "fraction of the image", not metres. At 10 m the camera sees a wide area, so a 0.1 error is a large distance. At 1 m the same 0.1 is a small distance. With a fixed gain the drone reacts strongly when high and weakly when low, so one setting is rarely good at every height.

**v2:** the error is converted to metres using the height and the camera field of view:

```
ground_error_m = normalised_error × height × tan(hfov / 2)
```
The speed is then `kp × ground_error_m`, limited to a maximum speed and a maximum acceleration, with a little smoothing on the camera readings. The result is the same behaviour at any height.

**Limits of this approach.** It assumes the camera points straight down and that the field of view in the file is right. It is a clean upgrade, not a perfect controller. If you later see small wobbles, the next step is adding a derivative term (a PD controller), which damps motion.

### 7.2 Aspect ratio

v1 divided `nx` by half the width but `ny` by half the height. On a 16:9 image, the same angle gave a vertical number about 1.8 times larger than the horizontal one. v2 divides both by half the width.

### 7.3 Yaw coupled to `nx` (and the sign problem)

v1 commanded `yaw_rate = -1.2 × nx` in addition to moving sideways with `vy = +Kp × nx`.

* **It double-corrects.** Moving sideways already fixes the horizontal error. Rotating as well turns the camera, which mixes the horizontal and vertical errors and can cause circling.
* **The sign looks wrong.** MAVSDK's `yawspeed_deg_s` is positive for clockwise rotation seen from above (turning right). A marker on the right (`nx > 0`) gave a *negative* value, which turns left, away from the marker. (The PDF says negative means clockwise; please check the MAVSDK documentation for your version.)
* **The size is tiny anyway.** The unit is degrees per second, so the command was about 1 degree per second at most. It would do little good, but it could add drift.
* **Landing on a marker does not need yaw.**

**v2:** yaw rate is always 0. The drone keeps its heading. If you ever want to match the marker's rotation, estimate it from the marker corners and add it as a separate, tested feature.

### 7.4 `DESCEND_RATE = 0.2` m/s

From 8 m this takes 40 seconds, which is slow where precision does not matter, and 0.2 m/s may still be fast right at the end. v2 sets the speed from height: fast when high, slow near the ground (`descent_gain × height`, kept between `descent_min_mps` and `descent_max_mps`). It also slows down (down to zero) if the drone drifts off-centre.

### 7.5 `ALIGN_THRESHOLD = 0.1`

0.1 of the half-image is 0.1 m at about 1 m height but about 0.5 m at 5 m, so "centred" meant different things at different heights. v2 uses a distance in metres (`align_threshold_m = 0.25`), requires the drone to **stay** centred for 1 s (`align_dwell_s`), and goes back to ALIGN if the error grows beyond `realign_threshold_m`.

### 7.6 `LAND_ALTITUDE_M = 0.5`

Handing over to PX4 LAND at 0.5 m is a reasonable choice. Two cautions:

* The height is measured from the start point, so check it in your own world.
* Near the ground the marker often cannot be seen. v2 adds `FINAL_DESCENT` below 1.5 m: straight down at slow speed, sideways correction only when the marker is visible.

v2 also waits for PX4 to confirm touchdown (`landed_state`) before it says it is done.

---

## 8. Other issues found

**In `camera_node.py`**

* **First marker only.** If two markers are visible, the code used `corners[0]`, which may be the wrong one. v2 uses `target_marker_id`.
* **`cv2.imshow` inside the callback** fails on computers without a display and slows every frame. v2 makes it optional (off by default).
* **A `print` per frame** floods the terminal. v2 uses throttled log messages.
* **Image shrinking by slicing** (`frame[::2, ::2]`) drops pixels and reduces the distance at which the marker can be seen. v2 uses `downscale: 1` by default and `cv2.resize` with area averaging if you need it.
* **No time stamp** in the error messages. v2 copies the camera frame's time stamp.
* **Sub-pixel corner refinement** turned on for a steadier marker centre.
* **QoS.** v2 uses the "sensor" setting so old frames are dropped instead of queued.

**In `precision_landing.py`**

* **Starting on the ground.** The code sent RTL without checking that the drone was flying. v2 checks first and explains what to do.
* **OFFBOARD failure** ended the thread and left the drone in whatever mode it had. v2 goes to ABORT and asks PX4 to HOLD.
* **Ctrl+C** left the drone without setpoints, so PX4's own OFFBOARD-loss failsafe took over. v2 stops cleanly into HOLD.
* **Loop timing** used `sleep(dt)` after the work, so the real rate was below 20 Hz. v2 schedules the next tick from a clock.
* **RTL may land by itself.** PX4's RTL ends with landing at home. If the vision system has not seen the marker in time, PX4 lands anyway. The runbook explains the RTL parameters to check.

**In the documentation**

* The README said `Gazebo Garden`. PX4 now supports newer Gazebo releases, and its setup script installs the supported one. The runbook explains how to check your version.
* The PDF describes the v1 topics and constants. It should be marked as v1 or regenerated (see `GIT_GUIDE.md`).
* Nothing explained that the drone must already be flying when the controller starts.

---

## 9. Still limited in v2 (be honest in your report)

* The camera is assumed to look straight down; drone tilt is not corrected.
* Height comes from PX4's local position. A real drone needs a rangefinder.
* Camera lens distortion is not corrected (no calibration), which matters on real cameras.
* There is no wind handling beyond what PX4 does.
* Only one marker is tracked. A nested (big and small) marker is the usual next step for the final metres.
* The field of view value must be measured for your camera.

These are good "future work" items for a project report.
