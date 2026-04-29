#!/usr/bin/env python3
"""
precision_landing.py — Control brain for Autonomous Precision Landing System.

Reads normalised ArUco alignment errors from camera_node.py and uses a
proportional controller + state machine to fly the drone over the marker
and land precisely on it via MAVSDK OFFBOARD velocity commands.

Subscribed topics:
    /aruco_error_x  (std_msgs/Float32) — horizontal error [-1.0, +1.0]
    /aruco_error_y  (std_msgs/Float32) — vertical error   [-1.0, +1.0]

State machine:
    RTL → OFFBOARD_INIT → ALIGN → DESCEND → (PX4 LAND)
"""

import asyncio
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


# ─── Tuning constants ─────────────────────────────────────────────────────────
KP                   = 0.6    # Proportional gain — increase for faster correction, decrease to reduce oscillation
KP_YAW_ALIGN        = 1.2    # Yaw gain during ALIGN state
KP_YAW_DESCEND      = 1.0    # Yaw gain during DESCEND state (slightly reduced for stability)
DESCEND_RATE         = 0.2    # Vertical descent speed in m/s (NED: positive = downward)
LAND_ALTITUDE_M      = 0.5    # Switch to PX4 LAND when altitude (metres) drops below this
ALIGN_THRESHOLD      = 0.1    # Normalised error magnitude — considered "centred" when |nx|,|ny| < this
STABLE_DETECT_SEC    = 3.0    # Seconds the marker must be continuously visible before OFFBOARD switch
DETECTION_TIMEOUT    = 0.5    # Seconds since last detection before resetting stability timer
OFFBOARD_PRESTREAM   = 50     # Number of zero-velocity setpoints to stream before starting OFFBOARD
MAVSDK_ADDRESS       = 'udp://:14540'
CONTROL_HZ           = 20     # Control loop rate in Hz


class PrecisionLanding(Node):
    """
    ROS2 node that subscribes to ArUco errors and drives the drone to land on
    the marker using MAVSDK in a separate async thread.
    """

    def __init__(self):
        super().__init__('precision_landing')

        # ── Shared state (written by ROS2 callbacks, read by MAVSDK thread) ──
        self.nx                   = 0.0    # Latest horizontal error
        self.ny                   = 0.0    # Latest vertical error
        self.last_detection_time  = 0.0    # Epoch timestamp of most recent detection
        self.first_detection_time = None   # Epoch timestamp when current detection sequence began
        self.state                = 'RTL'  # State machine initial state
        self.current_z            = 0.0    # Current NED down position (metres, positive = downward)

        # ── ROS2 subscribers ─────────────────────────────────────────────────
        self.create_subscription(Float32, '/aruco_error_x', self._x_cb, 10)
        self.create_subscription(Float32, '/aruco_error_y', self._y_cb, 10)

        # ── MAVSDK async loop runs in a background daemon thread ─────────────
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._start_loop, daemon=True).start()

    # ─── Subscriber callbacks ─────────────────────────────────────────────────

    def _x_cb(self, msg: Float32) -> None:
        """Update horizontal error and detection timestamps."""
        self.nx                  = msg.data
        self.last_detection_time = time.time()
        if self.first_detection_time is None:
            self.first_detection_time = time.time()

    def _y_cb(self, msg: Float32) -> None:
        """Update vertical error and detection timestamps."""
        self.ny                  = msg.data
        self.last_detection_time = time.time()
        if self.first_detection_time is None:
            self.first_detection_time = time.time()

    # ─── MAVSDK thread ────────────────────────────────────────────────────────

    def _start_loop(self) -> None:
        """Entry point for the background thread; runs the MAVSDK coroutine."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._run())

    async def _run(self) -> None:
        """Main async coroutine: connect to PX4, trigger RTL, then run state machine."""

        # ── Connect ───────────────────────────────────────────────────────────
        drone = System()
        await drone.connect(system_address=MAVSDK_ADDRESS)
        print('Connecting to PX4...')
        async for state in drone.core.connection_state():
            if state.is_connected:
                print('Connected')
                break

        # ── Trigger RTL ───────────────────────────────────────────────────────
        print('Triggering RTL...')
        await drone.action.return_to_launch()

        dt = 1.0 / CONTROL_HZ

        # ── Main control loop (≈20 Hz) ────────────────────────────────────────
        while True:

            # Read current altitude (NED down_m; negative when above ground)
            async for pos in drone.telemetry.position_velocity_ned():
                self.current_z = pos.position.down_m
                break

            now = time.time()

            # Detection freshness checks
            detected_recent = (now - self.last_detection_time) < DETECTION_TIMEOUT
            detected_stable = (
                self.first_detection_time is not None
                and (now - self.first_detection_time) > STABLE_DETECT_SEC
            )

            # Reset stability timer if marker lost
            if not detected_recent:
                self.first_detection_time = None

            # ── STATE: RTL ────────────────────────────────────────────────────
            if self.state == 'RTL':
                sys.stdout.write('\r[RTL] Returning home... ')
                sys.stdout.flush()
                if detected_stable:
                    print('\nMarker stable for 3 s → switching to OFFBOARD')
                    self.state = 'OFFBOARD_INIT'

            # ── STATE: OFFBOARD_INIT ──────────────────────────────────────────
            elif self.state == 'OFFBOARD_INIT':
                print('Pre-streaming setpoints...')
                for _ in range(OFFBOARD_PRESTREAM):
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
                    )
                    await asyncio.sleep(dt)
                try:
                    await drone.offboard.start()
                    print('OFFBOARD started')
                    self.state = 'ALIGN'
                except Exception as exc:
                    print(f'OFFBOARD failed: {exc}')
                    return

            # ── STATE: ALIGN ──────────────────────────────────────────────────
            elif self.state == 'ALIGN':
                vx       = -KP * self.ny           # forward/backward correction
                vy       = +KP * self.nx            # left/right correction
                vz       = 0.0                      # hold altitude
                yaw_rate = -KP_YAW_ALIGN * self.nx  # rotate to face marker

                sys.stdout.write(
                    f'\r[ALIGN]   nx={self.nx:+.2f}  ny={self.ny:+.2f}  '
                )
                sys.stdout.flush()

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
                )

                if abs(self.nx) < ALIGN_THRESHOLD and abs(self.ny) < ALIGN_THRESHOLD:
                    print('\nAligned → Descending')
                    self.state = 'DESCEND'

            # ── STATE: DESCEND ────────────────────────────────────────────────
            elif self.state == 'DESCEND':
                vx       = -KP * self.ny             # continue centering
                vy       = +KP * self.nx
                vz       = DESCEND_RATE              # controlled descent
                yaw_rate = -KP_YAW_DESCEND * self.nx

                altitude = -self.current_z            # convert NED to positive altitude
                sys.stdout.write(
                    f'\r[DESCEND] nx={self.nx:+.2f}  ny={self.ny:+.2f}  '
                    f'alt={altitude:.2f} m'
                )
                sys.stdout.flush()

                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
                )

                if abs(self.current_z) < LAND_ALTITUDE_M:
                    print('\nAltitude < 0.5 m → LAND')
                    await drone.action.land()
                    break

            await asyncio.sleep(dt)


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = PrecisionLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
