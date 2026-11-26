# SITL Runbook: Running the Precision Landing Simulation from Scratch

This guide takes you from a fresh Ubuntu 22.04 computer to a drone landing on a marker in the simulator. Follow the parts in order. Each part ends with a short check so you know it worked before moving on.

> **Tip:** Keep a notes file open. Simulator problems are usually a single wrong name (a topic, a world, a file path), so write down the exact names that work on your machine.

---

## Part 1: Install the tools (one time)

### 1.1 PX4 and its simulator

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```
Restart the computer when the script finishes.

PX4's setup script installs the Gazebo version that PX4 currently supports. Your v1 README also installed `gz-garden` separately. That is usually not needed and can create two Gazebo versions on one machine. Check what you have:

```bash
gz sim --versions
```

### 1.2 ROS 2 Humble

Follow the official ROS 2 Humble install page, then:
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 1.3 The ROS to Gazebo bridge and Python packages

```bash
sudo apt install ros-humble-ros-gz ros-humble-cv-bridge
pip install -r requirements.txt
```

> **If the bridge runs but no images arrive:** the `ros-humble-ros-gz` package is normally built for an older Gazebo release than the one PX4 installs. If your Gazebo (from `gz sim --versions`) is *Harmonic*, install `ros-humble-ros-gzharmonic` instead. Your v1 setup worked, so only change this if you hit this problem.

**Check:** `python3 -c "import cv2; print(cv2.__version__); cv2.aruco.ArucoDetector"` prints a version of 4.7 or higher with no error.

---

## Part 2: Start the simulator

Open a terminal and run:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down
```

This starts PX4 and Gazebo with the x500 drone that has a camera pointing straight down. The first build takes a few minutes.

For the marker world that PX4 provides, use:
```bash
PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down
```

**Check:** A Gazebo window opens and the terminal shows a `pxh>` prompt.

### Optional: make RTL less aggressive

PX4's default RTL (return to launch) flies home at a high altitude and then lands by itself. A marker looks tiny from high up. In the `pxh>` prompt you can lower the heights:

```
param set RTL_RETURN_ALT 10
param set RTL_DESCEND_ALT 8
```
Also look at `RTL_LAND_DELAY` in QGroundControl's parameter list and read its description: it controls how long PX4 waits before landing on its own, which is the time your vision node has to spot the marker. (Parameter names and defaults can change between PX4 versions, so confirm them in QGroundControl.)

---

## Part 3: Find the camera topic and set up the bridge

Gazebo and ROS 2 use different topic names. The bridge copies one to the other.

### 3.1 Find the Gazebo topic

In a **new** terminal:
```bash
gz topic -l | grep -E "image|camera_info"
```
You should see something like:
```
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info
```
The world name (`default`) and the sensor name (`camera`) can differ between PX4 versions. **Use exactly what your machine prints.** If you used the `aruco` world, `default` will say `aruco`.

### 3.2 Start the bridge

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  --ros-args -r /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image:=/downcam/image_raw
```

What each part does:

| Part | Meaning |
|---|---|
| `.../camera/image` | The Gazebo topic (from step 3.1) |
| `@sensor_msgs/msg/Image` | The ROS 2 message type |
| `[gz.msgs.Image` | The Gazebo type; `[` means one-way, Gazebo to ROS 2 |
| `-r old:=new` | Renames the long topic to the short `/downcam/image_raw` |
| Quotes | Needed so the terminal does not treat `[` as a special character |

### 3.3 Check the images arrive

```bash
ros2 topic hz /downcam/image_raw
```
You should see a steady rate (for example 30 Hz). If you see nothing, see "Camera troubleshooting" below.

---

## Part 4: Find the camera field of view

The controller converts "pixels off centre" into "metres off centre", so it needs the camera's field of view. Run:

```bash
gz topic -e -t /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info -n 1
```
Find `width` and the first number of the `k` matrix (`fx`). Then:

```
hfov = 2 × atan( width / (2 × fx) )     (result in radians)
```

Example with Python:
```bash
python3 -c "import math; print(2*math.atan(640/(2*277.0)))"
```
(replace 640 and 277.0 with your numbers). Put the result into `camera_hfov_rad` in `config/params.yaml`. The default `1.74` is only a starting guess.

---

## Part 5: Start the vision node

```bash
source /opt/ros/humble/setup.bash
cd ~/precision-landing-PX4
python3 src/camera_node.py --ros-args --params-file config/params.yaml
```

Expected output: `Aruco detector started ...`, then either `marker not detected` or `marker 0: nx=... ny=...`.

To watch the camera view with the marker outline:
```bash
ros2 run rqt_image_view rqt_image_view /camera_debug
```

**Check:** In another terminal, `ros2 topic echo /aruco_error` prints values when the marker is in view.

---

## Part 6: Take off and run the controller

### 6.1 Take off

In the PX4 terminal (`pxh>`):
```
commander takeoff
```
Wait until the drone hovers.

### 6.2 Put the drone and marker in a good starting position

Pick one:

* **Scenario A (easiest first test):** the marker is under or near the drone. In `config/params.yaml` set `start_with_rtl: false`. The controller will skip RTL and wait for the marker to be visible for 3 seconds.
* **Scenario B (full demo):** keep `start_with_rtl: true`. Fly the drone away first (in QGroundControl, right-click the map and choose *Go to location*). The controller then triggers RTL and watches for the marker on the way back. The marker must be near the home point.

### 6.3 Start the controller

```bash
source /opt/ros/humble/setup.bash
cd ~/precision-landing-PX4
python3 src/precision_landing.py --ros-args --params-file config/params.yaml
```

You will see log lines like:
```
State: WAIT_FOR_MARKER -> OFFBOARD_INIT
OFFBOARD started
State: OFFBOARD_INIT -> ALIGN
[ALIGN] err=0.62 m  height=8.0 m
State: ALIGN -> DESCEND
[DESCEND] err=0.10 m  height=4.3 m  vz=0.50
State: FINAL_DESCENT -> LAND
Touchdown confirmed
```

**Emergency stop:** press `Ctrl+C` in the controller terminal. It stops OFFBOARD and tells PX4 to hold position. You can also switch modes in QGroundControl.

---

## Part 7: The sign test (do this once)

The signs of the movement depend on how the camera is mounted. Test them before trusting a landing.

1. Start the system as in Part 6.
2. In the first seconds of `ALIGN`, watch the `err=` value in the log.
3. If `err` **shrinks**, the signs are right.
4. If `err` **grows** (the drone flies away from the marker):
   * Moves away **forward/back**: change `sign_forward` to `1.0`.
   * Moves away **left/right**: change `sign_right` to `-1.0`.
   * Stop with `Ctrl+C`, edit `params.yaml`, and try again.

---

## Part 8: Run the offline tests

No ROS 2 or PX4 needed:
```bash
python3 tests/test_aruco_detection.py
```

---

## Shutdown order

1. `Ctrl+C` in the controller terminal.
2. `Ctrl+C` in the vision node and bridge terminals.
3. `Ctrl+C` in the PX4 terminal (this also closes Gazebo).

---

## Camera troubleshooting

| Symptom | Fix |
|---|---|
| `gz topic -l` shows no image topic | PX4 is not running with the camera model. Use exactly `gz_x500_mono_cam_down`. |
| Image topic exists but the bridge is silent | Use `gz topic -e -t <topic> -n 1` to confirm Gazebo is publishing. If it is, the ROS side usually needs a matching Gazebo bridge package (see part 1.3). |
| `ros2 topic hz` prints nothing | The bridge command's topic does not match `gz topic -l` exactly (world name, model number, sensor name). |
| Camera view is empty grey | There is no marker in the world below the drone. Add it in Gazebo or use the `aruco` world. |
| Window crashes at start | Keep `show_window: false` and use `rqt_image_view /camera_debug`. |
| Marker found only close up | Marker too small or image too small: print it larger, or keep `downscale: 1`. |
| `ParameterTypeException` | A decimal setting lost its decimal point in `params.yaml`. |

---

## Notes on accuracy

* The controller's descent height comes from PX4's local position (height above where the drone started). On flat ground in the simulator this is fine. On real hardware you need a proper height source.
* The pixel-to-metre conversion assumes the camera points straight down. Tilt in the drone's body adds a small error that shrinks as the drone steadies.
* The controller keeps the drone's heading fixed. It does not rotate to match the marker.
