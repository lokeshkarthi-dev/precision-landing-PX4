# Autonomous Vision-Based Precision Landing System

A fully autonomous precision landing system for multirotor drones.  
The drone detects an **ArUco marker** on the ground using a downward-facing camera and lands precisely on it — no human input required.

**Stack:** PX4 Autopilot · QGroundControl · Gazebo · ROS2 Humble · Python · MAVSDK · OpenCV

---

## 📚 System Documentation

📖 **[Precision Landing for Multirotors - Complete Documentation](Precision_Landing_for_Multirotors.pdf)**

---

## How It Works

```
Gazebo (virtual world + camera)
    ↓
PX4 SITL (autopilot running in software)
    ↓
ros_gz_bridge (Gazebo image → ROS2 topic)
    ↓
camera_node.py (ArUco detection → publishes nx, ny errors)
    ↓
precision_landing.py (reads errors → MAVSDK velocity commands)
    ↓
PX4 SITL (executes commands → updates drone position)
    ↓  (loop repeats)
```

### State Machine

| State | Description |
|---|-----------|
| **RTL** | Drone returns to launch. Waits for marker to be visible for 3 continuous seconds. |
| **OFFBOARD_INIT** | Pre-streams zero-velocity setpoints to satisfy PX4's OFFBOARD requirement. |
| **ALIGN** | Corrects horizontal position over the marker. No descent yet. |
| **DESCEND** | Continues position correction AND descends at 0.2 m/s. |
| **LAND** | Triggered when altitude < 0.5 m. PX4 handles final touchdown. |

---

## Repository Structure

```
precision-landing/
├── src/
│   ├── camera_node.py        # Perception: ArUco detection, publishes errors
│   └── precision_landing.py  # Control: state machine + MAVSDK velocity commands
├── config/
│   └── params.yaml           # All tunable parameters (gains, thresholds, etc.)
├── assets/
│   ├── generate_marker.py    # Utility to generate printable ArUco markers
│   └── markers/              # Generated marker images go here
├── tests/
│   └── test_aruco_detection.py  # Offline smoke tests (no ROS2/PX4 required)
├── docs/
│   └── (additional documentation)
├── requirements.txt
├── Precision_Landing_for_Multirotors.pdf
└── README.md
```

---

## Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **Python:** 3.10+

---

## Installation

### 1 — PX4 SITL

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

### 2 — Gazebo & Gazebo Garden

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install Gazebo
sudo apt update
sudo apt install -y gz-garden

# Verify installation
gz --version
```

### 3 — ROS2 Humble

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt install ros-humble-desktop
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 4 — Project dependencies

```bash
# System packages
sudo apt install ros-humble-ros-gz ros-humble-cv-bridge

# Python packages
pip install -r requirements.txt
```

---

## Running the System

Open **four terminals** and run each command in order:

**Terminal 1 — PX4 SITL + Gazebo**
```bash
cd PX4-Autopilot
make px4_sitl gz_x500_gimbal
```

**Terminal 2 — ROS–Gazebo bridge**
```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /gimbal_camera@sensor_msgs/msg/Image@gz.msgs.Image
```

**Terminal 3 — Vision node**
```bash
source /opt/ros/humble/setup.bash
python3 src/camera_node.py
```

**Terminal 4 — Precision landing controller**
```bash
source /opt/ros/humble/setup.bash
python3 src/precision_landing.py
```

The drone will connect, trigger RTL, and once the marker is visible for 3 seconds it switches to OFFBOARD mode and begins the precision landing sequence.

---

## Generate a Printable Marker

```bash
python3 assets/generate_marker.py           # marker ID 0, 600 px
python3 assets/generate_marker.py --id 3 --size 800
```

Print at **≥ 15 cm × 15 cm** for reliable detection at 5 m altitude.

---

## Running Tests

No ROS2 or PX4 needed — just OpenCV and NumPy:

```bash
python3 tests/test_aruco_detection.py
```

---

## Tuning

All parameters are in [`config/params.yaml`](config/params.yaml).

| Parameter | Default | Effect |
|---|---|---|
| `kp` | `0.6` | Proportional gain — increase for faster correction, reduce to stop oscillation |
| `descend_rate_mps` | `0.2` | Descent speed in m/s — lower = more precise |
| `align_threshold` | `0.1` | How centred is "centred" before descent begins |
| `stable_detect_sec` | `3.0` | Seconds of stable detection before switching to OFFBOARD |
| `land_altitude_m` | `0.5` | Altitude at which PX4 LAND is triggered |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| **OFFBOARD rejected** | Pre-stream setpoints for ≥ 2 s before calling `offboard.start()` |
| **Marker not detected** | Check `/gimbal_camera` topic is publishing; ensure marker is well-lit and printed clearly |
| **Drone drifts in ALIGN** | Reduce `kp` (try `0.3`); high gain causes oscillation |
| **MAVSDK connection refused** | PX4 SITL must be running first; default port is `udp://:14540` |
| **ROS2 topic not found** | Ensure `ros_gz_bridge` is running and topic name matches exactly |
| **Drone overshoots** | Reduce `descend_rate_mps` (try `0.1`) or lower `land_altitude_m` |
| **imshow window missing** | Headless environment — comment out `imshow()` and use `rqt_image_view /camera_debug` |
| **Gazebo not found** | Verify Gazebo installation with `gz --version` and ensure `gz-garden` is installed |

---

## Real Hardware

The same scripts work on real hardware with one change in `src/precision_landing.py`:

```python
# SITL (default)
await drone.connect(system_address='udp://:14540')

# Real hardware via USB/serial
await drone.connect(system_address='serial:///dev/ttyUSB0:57600')
```

> ⚠️ Always test every change in SITL simulation before deploying to a real drone.

---

## Key Concepts

| Term | Meaning |
|---|---|
| **PX4** | Open-source autopilot — the drone's brain |
| **QGroundControl** | Operator dashboard app |
| **SITL** | Software-In-The-Loop — full simulation in software, no hardware needed |
| **OFFBOARD mode** | PX4 mode where an external computer sends velocity commands |
| **ArUco marker** | Printed square barcode used as visual landing target |
| **Normalised error** | Marker offset scaled to [-1.0, +1.0] relative to image centre |
| **P-controller** | `command = Kp × error` |
| **NED frame** | North-East-Down coordinates — positive Z is downward |
| **cv_bridge** | Converts between ROS2 Image messages and OpenCV arrays |
