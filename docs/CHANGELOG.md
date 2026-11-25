# Changelog

## v2.0.0: Downward monocam, safer and smoother controller

Version 1 used the gimbal camera model (`gz_x500_gimbal`). Version 2 uses a fixed downward-facing camera and fixes the main safety and stability issues found in the review ([`CODE_AUDIT.md`](CODE_AUDIT.md)).

### What changed and what improved

#### Simulator and camera

| | v1 | v2 | Why it is better |
|---|---|---|---|
| PX4 target | `make px4_sitl gz_x500_gimbal` | `make px4_sitl gz_x500_mono_cam_down` | A fixed camera that points straight down, which is what landing needs. No gimbal angle to worry about. |
| Camera topic (Gazebo) | `/gimbal_camera` | `/world/<world>/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image` | This is the real name PX4 creates. Check yours with `gz topic -l`. |
| Camera topic (ROS 2) | `/gimbal_camera` | `/downcam/image_raw` (renamed in the bridge command) | A short stable name, so the code does not depend on the world name. |
| Bridge command | `parameter_bridge /gimbal_camera@sensor_msgs/msg/Image@gz.msgs.Image` | Long topic with `[` (one-way) and a `-r` rename | One-way is safer, and the rename keeps the code simple. |

#### `camera_node.py` (perception)

| | v1 | v2 | Why it is better |
|---|---|---|---|
| Output | Two topics: `/aruco_error_x`, `/aruco_error_y` | One topic: `/aruco_error` (`PointStamped`) | x and y always arrive together; carries the camera time stamp. |
| `ny` scale | Divided by half the height | Divided by half the width | Same scale in both directions on a wide image. |
| Marker choice | First marker found | Marker ID from `target_marker_id` | Cannot lock onto the wrong marker. |
| Corner accuracy | Default | Sub-pixel refinement | Steadier centre position. |
| Image shrinking | Slicing `[::2, ::2]` | `downscale` setting with area averaging (default: none) | Keeps more detail, so detection works from further away. |
| QoS | Default queue of 10 | Sensor QoS (newest frame only) | No growing delay when processing is slow. |
| Window and printing | `imshow` and `print` on every frame | Off by default; throttled log messages; debug image 1 in 3 frames | Works without a display; much less CPU and terminal noise. |
| Settings | Hard-coded | ROS 2 parameters from `params.yaml` | Tune without editing code. |

#### `precision_landing.py` (control)

| | v1 | v2 | Why it is better |
|---|---|---|---|
| Sharing data between threads | Plain variables, no lock | One read-only reading swapped in under a lock | No half-updated values or timer races. |
| Timing | `time.time()` | `time.monotonic()` | Not affected by clock changes. |
| Telemetry | New stream opened on each loop pass | One background task keeps the latest value | Less overhead and a freshness check. |
| Marker lost | Used old errors forever | LOST state: hover, climb slowly, retry, then HOLD | Biggest safety improvement. |
| Error units | Fraction of image | Metres on the ground (uses height and field of view) | Same behaviour at every height. |
| Speed command | Raw P-control | Smoothed errors, maximum speed, acceleration limit | No jerky movements. |
| Yaw | `-1.2 × nx` (wrong sign, fights the sideways move) | Always 0 (heading held) | Removes a source of drift and circling. |
| Alignment | `abs(err) < 0.1` in image units, once | Metres, held for 1 s, with a way back from DESCEND | Same meaning at any height; no blind commitment. |
| Descent speed | Fixed 0.2 m/s | Faster high up, slower near the ground, slows when off-centre | Quicker landing without losing accuracy. |
| Close to the ground | Same logic all the way | `FINAL_DESCENT` below 1.5 m | Handles the marker leaving the view. |
| End of flight | Called LAND and exited | Waits for touchdown confirmation, exits cleanly | Clear result and clean shutdown. |
| Before start | Assumed the drone was flying | Checks the drone is flying and has altitude data | Clear error message instead of a confusing failure. |
| Errors and Ctrl+C | Thread could die silently; Ctrl+C left no setpoints | Errors logged, ABORT to HOLD, Ctrl+C holds position | Safer behaviour on any problem. |
| Loop timing | `sleep(dt)` after the work | Clock-based schedule | Closer to a true 20 Hz. |
| Settings | Hard-coded; `params.yaml` unused | ROS 2 parameters from `params.yaml` | Config file now does what the docs say. |
| Output | `print` with `\r` | ROS 2 logger with throttling | Works in logs and remote terminals. |

#### State machine

| v1 | v2 |
|---|---|
| `RTL → OFFBOARD_INIT → ALIGN → DESCEND → (LAND)` | `WAIT_FOR_MARKER → OFFBOARD_INIT → ALIGN ⇄ DESCEND → FINAL_DESCENT → LAND → DONE`, plus `LOST` and `ABORT` |

#### Settings (`params.yaml`)

| v1 constant | v2 setting | Note |
|---|---|---|
| `KP = 0.6` | `kp: 0.5` | Now in 1/second on metres, not on image fractions |
| `DESCEND_RATE = 0.2` | `descent_gain`, `descent_min_mps`, `descent_max_mps` | Depends on height |
| `ALIGN_THRESHOLD = 0.1` | `align_threshold_m: 0.25` (+ `align_dwell_s`, `realign_threshold_m`) | In metres |
| `LAND_ALTITUDE_M = 0.5` | `land_altitude_m: 0.5` | Same value |
| `STABLE_DETECT_SEC = 3.0` | `stable_detect_s: 3.0` | Same value |
| `DETECTION_TIMEOUT = 0.5` | `detection_timeout_s: 0.5` | Same value |
| `KP_YAW_ALIGN`, `KP_YAW_DESCEND` | removed | Yaw control removed |
| `OFFBOARD_PRESTREAM = 50` | `prestream_s: 2.5` | Time instead of a count |
| `MAVSDK_ADDRESS` | `mavsdk_address` | Same default |
| (new) | `camera_hfov_rad`, `sign_forward`, `sign_right`, `filter_alpha`, `lost_*`, `final_descent_alt_m`, `telemetry_*` | New features |

#### Documentation

* New runbook with a camera-topic check, field-of-view measurement and a sign test.
* New audit, changelog and Git guide.
* README rewritten for v2.
* The v1 PDF describes the old design; keep it only as a reference (see the Git guide).

### Things you must do after upgrading

1. Use `make px4_sitl gz_x500_mono_cam_down` and the new bridge command.
2. Start both nodes with `--ros-args --params-file config/params.yaml`.
3. Measure the camera field of view and update `camera_hfov_rad`.
4. Run the sign test (runbook, Part 7).
5. Take off **before** starting the controller.

### Known limits

See section 9 of [`CODE_AUDIT.md`](CODE_AUDIT.md).

---

## v1.0.0: Gimbal camera version

First working version: ArUco detection with a gimbal camera, proportional controller and a five-state machine in PX4 SITL.
