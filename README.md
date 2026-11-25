# Autonomous Vision-Based Precision Landing (v2, Downward Monocam)

A multirotor drone finds an **ArUco marker** on the ground with a downward-facing camera, centres itself above it, and lands on it without human input. It runs in simulation (PX4 SITL + Gazebo) using only open-source tools.

**Stack:** PX4 Autopilot · Gazebo · ROS 2 Humble · Python · MAVSDK · OpenCV · QGroundControl (optional monitor)

> **Version 2** replaces the gimbal camera (`gz_x500_gimbal`) with a fixed downward-facing camera (`gz_x500_mono_cam_down`) and makes the controller safer and smoother. See [`docs/CHANGELOG.md`](docs/CHANGELOG.md) for every change, and [`docs/CODE_AUDIT.md`](docs/CODE_AUDIT.md) for the reasons behind them.

---

## How it works

```
Gazebo (world + camera)
   │  image
   ▼
ros_gz_bridge  ──►  /downcam/image_raw
   │
   ▼
camera_node.py       finds the marker, publishes /aruco_error (nx, ny)
   │
   ▼
precision_landing.py turns the error into velocity commands (MAVSDK, OFFBOARD mode)
   │
   ▼
PX4 SITL  ──►  moves the drone in Gazebo  ──►  new camera image (loop repeats)
```

* `camera_node.py` is the **eyes**: it only looks for the marker and reports where it is.
* `precision_landing.py` is the **brain**: it decides how to move.
* Keeping them separate means each one can be tested on its own.

### The error message

`/aruco_error` (`geometry_msgs/PointStamped`) carries two numbers:

| Value | Meaning |
|---|---|
| `x` (`nx`) | Marker offset to the right of the image centre (negative = left) |
| `y` (`ny`) | Marker offset below the image centre (negative = above) |

Both are divided by half the image width, so one unit means the same angle in both directions. Nothing is published when the marker is not seen.

---

## State machine

```
WAIT_FOR_MARKER ─► OFFBOARD_INIT ─► ALIGN ◄──► DESCEND ─► FINAL_DESCENT ─► LAND ─► DONE
                                       ▲           │
                                       └── LOST ◄──┘   marker disappeared
                                             │
                                           ABORT       gave up: PX4 HOLD
```

| State | What the drone does | Moves on when |
|---|---|---|
| **WAIT_FOR_MARKER** | Flies PX4 RTL (return to launch) and watches for the marker | Marker seen for 3 s in a row |
| **OFFBOARD_INIT** | Sends zero-speed commands for 2.5 s, then starts OFFBOARD mode | OFFBOARD accepted (else ABORT) |
| **ALIGN** | Moves sideways/forward over the marker, holds height | Within 0.25 m of the marker for 1 s |
| **DESCEND** | Keeps centring while going down; descent slows if the marker drifts off-centre | Height below 1.5 m (or error above 0.8 m goes back to ALIGN) |
| **LOST** | Marker disappeared: hover 1 s, then climb slowly to see more ground | Marker seen again for 0.5 s (back to ALIGN) or 8 s passes (ABORT) |
| **FINAL_DESCENT** | Straight down at slow speed (the marker may fill or leave the view this close) | Height below 0.5 m |
| **LAND** | Hands over to PX4 LAND and waits for touchdown | Touchdown confirmed |
| **ABORT** | Stops OFFBOARD and asks PX4 to hold position | Pilot takes over |

---

## Repository structure

```
precision-landing-PX4/
├── src/
│   ├── camera_node.py          # Perception: marker detection
│   └── precision_landing.py    # Control: state machine + MAVSDK commands
├── config/
│   └── params.yaml             # Every tunable setting (now actually loaded!)
├── assets/
│   ├── generate_marker.py      # Makes printable ArUco markers
│   └── markers/
├── tests/
│   └── test_aruco_detection.py # Offline smoke tests (no ROS 2 / PX4 needed)
├── docs/
│   ├── SITL_RUNBOOK.md         # Step-by-step simulator guide
│   ├── CODE_AUDIT.md           # Honest review of v1
│   ├── CHANGELOG.md            # What changed and why
│   ├── GIT_GUIDE.md            # How to upload this version to GitHub
│   └── v1/                     # (optional) old PDF kept for reference
├── requirements.txt
├── .gitignore
└── README.md
```

---

## Prerequisites

* Ubuntu 22.04 LTS
* Python 3.10+
* PX4-Autopilot (with its Gazebo simulator), ROS 2 Humble, `ros_gz_bridge`

Full install steps are in [`docs/SITL_RUNBOOK.md`](docs/SITL_RUNBOOK.md).

---

## Quick start (four terminals)

Run them in this order. Use the same Gazebo world name everywhere (shown here as `default`).

**Terminal 1: PX4 + Gazebo with the downward monocam**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down
```
For the marker world that ships with PX4, use `PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down` (the world name in the topic below then becomes `aruco`).

**Terminal 2: camera bridge**
```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  --ros-args -r /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image:=/downcam/image_raw
```
Check the exact topic first with `gz topic -l | grep image` (see the runbook).

**Terminal 3: vision node**
```bash
source /opt/ros/humble/setup.bash
cd ~/precision-landing-PX4
python3 src/camera_node.py --ros-args --params-file config/params.yaml
```

**Terminal 4: take off, then start the controller**
In the PX4 console (Terminal 1) type `commander takeoff`, wait until the drone hovers, then:
```bash
source /opt/ros/humble/setup.bash
cd ~/precision-landing-PX4
python3 src/precision_landing.py --ros-args --params-file config/params.yaml
```

The controller refuses to start if the drone is on the ground. This is on purpose.

---

## Tuning

Everything is in [`config/params.yaml`](config/params.yaml). Change one value at a time.

| Parameter | Default | Effect |
|---|---|---|
| `kp` | `0.5` | Higher = faster centring. Too high = wobbling. |
| `max_horizontal_speed_mps` | `1.0` | Speed cap while centring. |
| `align_threshold_m` | `0.25` | How close (metres) counts as "centred". |
| `descent_gain`, `descent_min_mps`, `descent_max_mps` | `0.15`, `0.15`, `0.5` | Descent is faster when high and slower near the ground. |
| `final_descent_alt_m` | `1.5` | Below this the drone goes straight down. |
| `land_altitude_m` | `0.5` | Height where PX4 LAND takes over. |
| `camera_hfov_rad` | `1.74` | Camera field of view. **Check it for your camera.** |
| `sign_forward`, `sign_right` | `-1.0`, `1.0` | Flip if the drone moves away from the marker. |
| `lost_abort_s` | `8.0` | Seconds without the marker before giving up. |

---

## Troubleshooting

| Problem | What to try |
|---|---|
| `The drone is not flying` | Take off first (`commander takeoff` in the PX4 console). |
| No camera image in ROS 2 | Check `gz topic -l \| grep image`; the topic in the bridge command must match exactly, including the world name. |
| Marker never detected | View `/camera_debug` with `rqt_image_view`; check lighting and marker size; check `target_marker_id`. |
| Drone moves **away** from the marker | Flip `sign_forward` or `sign_right` in `params.yaml` (see runbook, sign test). |
| Drone wobbles around the marker | Lower `kp` (try `0.3`) or lower `filter_alpha`. |
| OFFBOARD rejected | Make sure the drone is flying and PX4 is running; the node already pre-streams for 2.5 s. |
| `ParameterTypeException` at start | A decimal setting in `params.yaml` was written as a whole number (write `1.0`, not `1`). |
| MAVSDK connection refused | Start PX4 SITL first; the default address is `udp://:14540`. |
| No window opens | Expected: the window is off by default. Use `rqt_image_view /camera_debug`. |

---

## Real hardware

The same scripts can talk to a real flight controller by changing one line in `config/params.yaml`:

```yaml
mavsdk_address: serial:///dev/ttyUSB0:57600
```

**Do not fly real hardware with these scripts as they are.** In simulation the height comes from PX4's local position estimate; a real drone needs a rangefinder or a well-tested height source, the camera needs calibrating, and the field of view must be measured. Test every change in the simulator first, fly with a safety pilot, and keep a way to take manual control.

---

## Key terms

| Term | Meaning |
|---|---|
| **PX4** | Open-source autopilot software, the drone's flight brain |
| **SITL** | Software-In-The-Loop: the whole drone simulated on a computer |
| **OFFBOARD mode** | PX4 mode where an outside computer sends velocity commands |
| **ArUco marker** | A printed black-and-white square the camera can recognise |
| **Normalised error** | The marker's offset from the image centre, scaled by half the image width |
| **P-controller** | Speed = gain × distance. Far away: fast. Close: slow. |
| **NED frame** | North-East-Down axes; positive Z points *down* |
| **ROS 2 parameter** | A setting that a node reads when it starts (here from `params.yaml`) |
