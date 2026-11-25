#!/usr/bin/env python3
"""
precision_landing.py - Control layer (v2) for the Autonomous Precision Landing System.

Reads the marker offset from camera_node.py and flies the drone over the marker
and down to a landing using MAVSDK OFFBOARD velocity commands.

Subscribed topic
----------------
/aruco_error (geometry_msgs/PointStamped)   nx = point.x, ny = point.y

State machine
-------------
WAIT_FOR_MARKER -> OFFBOARD_INIT -> ALIGN <-> DESCEND -> FINAL_DESCENT -> LAND -> DONE
                                      ^          |
                                      +-- LOST --+      (marker lost: hover, climb, retry)
                                            |
                                          ABORT         (gave up: HOLD position)

Threading model (important for beginners)
-----------------------------------------
Two threads run side by side:
  * Main thread   : rclpy.spin_once() -> receives marker errors (ROS 2 callback)
  * MAVSDK thread : asyncio loop      -> talks to PX4 and runs the state machine
The ONLY data passed between them is the newest marker reading, and it is
protected by a lock. Everything else is touched by the MAVSDK thread alone.

All settings are ROS 2 parameters, loaded from config/params.yaml:
    python3 src/precision_landing.py --ros-args --params-file config/params.yaml
"""

import asyncio
import math
import threading
import time
import traceback
from dataclasses import dataclass
from enum import Enum, auto
from types import SimpleNamespace
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import LandedState


# ---------------------------------------------------------------------------
# Default settings. params.yaml overrides these. The type of each default
# (float / bool / str) is also used to convert the value read from the file.
# ---------------------------------------------------------------------------
DEFAULTS = {
    'mavsdk_address': 'udp://:14540',
    'error_topic': '/aruco_error',
    'start_with_rtl': True,
    'control_hz': 20.0,
    'detection_timeout_s': 0.5,
    'stable_detect_s': 3.0,
    'reacquire_s': 0.5,
    'filter_alpha': 0.4,
    'camera_hfov_rad': 1.74,
    'min_height_for_scale_m': 0.3,
    'kp': 0.5,
    'max_horizontal_speed_mps': 1.0,
    'max_horizontal_accel_mps2': 1.0,
    'sign_forward': -1.0,
    'sign_right': 1.0,
    'align_threshold_m': 0.25,
    'align_dwell_s': 1.0,
    'realign_threshold_m': 0.8,
    'descent_gain': 0.15,
    'descent_min_mps': 0.15,
    'descent_max_mps': 0.5,
    'max_vertical_accel_mps2': 0.5,
    'final_descent_alt_m': 1.5,
    'land_altitude_m': 0.5,
    'lost_hover_s': 1.0,
    'lost_abort_s': 8.0,
    'recover_climb_mps': 0.3,
    'max_recover_alt_m': 15.0,
    'telemetry_rate_hz': 20.0,
    'telemetry_timeout_s': 1.0,
    'prestream_s': 2.5,
    'land_timeout_s': 60.0,
}


class State(Enum):
    WAIT_FOR_MARKER = auto()
    OFFBOARD_INIT = auto()
    ALIGN = auto()
    DESCEND = auto()
    LOST = auto()
    FINAL_DESCENT = auto()
    LAND = auto()
    DONE = auto()
    ABORT = auto()


@dataclass(frozen=True)
class Detection:
    """One marker reading. 'frozen' = cannot be changed after creation, so
    swapping in a new one is safe between threads."""
    nx: float
    ny: float
    t: float          # time.monotonic() when it was received


@dataclass
class Perception:
    """What the controller knows about the marker right now."""
    fresh: bool       # seen within detection_timeout_s
    streak_s: float   # how long it has been continuously visible
    nx: float         # smoothed errors (0.0 if not fresh)
    ny: float


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _step_towards(current: float, target: float, max_delta: float) -> float:
    """Move 'current' toward 'target' by at most 'max_delta' (acceleration limit)."""
    return current + _clip(target - current, -max_delta, max_delta)


class PrecisionLanding(Node):

    def __init__(self) -> None:
        super().__init__('precision_landing')

        # ---- Settings -------------------------------------------------------
        self.cfg = SimpleNamespace()
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            setattr(self.cfg, name, type(default)(value))

        # ---- Shared between the two threads (guarded by a lock) --------------
        self._lock = threading.Lock()
        self._latest: Optional[Detection] = None

        # ---- Thread control --------------------------------------------------
        self.finished = threading.Event()      # set when the MAVSDK thread is done
        self._stop_requested = threading.Event()

        # ---- State used ONLY by the MAVSDK thread ----------------------------
        self.state = State.WAIT_FOR_MARKER
        self.alt_m: Optional[float] = None     # height above the start point, metres
        self.alt_time = 0.0
        self.landed_state = None
        self._streak_start: Optional[float] = None
        self._last_det_t: Optional[float] = None
        self._fx: Optional[float] = None       # smoothed nx
        self._fy: Optional[float] = None       # smoothed ny
        self._cmd = (0.0, 0.0, 0.0)            # last velocity sent (vx, vy, vz)
        self._aligned_since: Optional[float] = None
        self._lost_since = 0.0

        self.create_subscription(PointStamped, self.cfg.error_topic, self._on_error, 5)

        # ---- Start the MAVSDK thread ----------------------------------------
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._thread_main, daemon=True)
        self._thread.start()

    # ======================================================================
    # ROS side (main thread)
    # ======================================================================
    def _on_error(self, msg: PointStamped) -> None:
        det = Detection(msg.point.x, msg.point.y, time.monotonic())
        with self._lock:
            self._latest = det

    def request_stop(self) -> None:
        """Called from main() on Ctrl+C: asks the controller to HOLD and exit."""
        self._stop_requested.set()
        self._thread.join(timeout=5.0)

    # ======================================================================
    # MAVSDK side (background thread)
    # ======================================================================
    def _thread_main(self) -> None:
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._run())
        except Exception:
            self.get_logger().error('Controller crashed:\n' + traceback.format_exc())
        finally:
            self.finished.set()

    def _goto(self, new_state: State) -> None:
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state

    def _status(self, text: str) -> None:
        self.get_logger().info(text, throttle_duration_sec=1.0)

    # ----------------------------------------------------------------------
    async def _run(self) -> None:
        cfg = self.cfg
        drone = System()
        await drone.connect(system_address=cfg.mavsdk_address)
        self.get_logger().info('Connecting to PX4...')
        async for conn in drone.core.connection_state():
            if conn.is_connected:
                self.get_logger().info('Connected to PX4')
                break

        # Telemetry is read by background tasks that keep the newest values.
        # (v1 opened a brand-new telemetry stream on EVERY loop pass.)
        tasks = [
            asyncio.create_task(self._watch_position(drone)),
            asyncio.create_task(self._watch_landed(drone)),
        ]
        try:
            await self._preflight(drone)
            await self._control_loop(drone)
        except Exception:
            self.get_logger().error('Error in control loop:\n' + traceback.format_exc())
            self.state = State.ABORT
        finally:
            if self.state is State.ABORT:
                await self._abort(drone)
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    # ----------------------------------------------------------------------
    async def _watch_position(self, drone: System) -> None:
        async for pv in drone.telemetry.position_velocity_ned():
            # NED: "down" is positive below the start point, so height = -down.
            self.alt_m = -pv.position.down_m
            self.alt_time = time.monotonic()

    async def _watch_landed(self, drone: System) -> None:
        async for s in drone.telemetry.landed_state():
            self.landed_state = s

    # ----------------------------------------------------------------------
    async def _preflight(self, drone: System) -> None:
        cfg = self.cfg
        try:
            await drone.telemetry.set_rate_position_velocity_ned(cfg.telemetry_rate_hz)
        except Exception as exc:     # not fatal: default rate still works
            self.get_logger().warn(f'Could not set telemetry rate: {exc}')

        in_air = False
        async for flying in drone.telemetry.in_air():
            in_air = flying
            break
        if not in_air:
            raise RuntimeError('The drone is not flying. Take off first (see docs/SITL_RUNBOOK.md).')

        deadline = time.monotonic() + 5.0
        while self.alt_m is None and time.monotonic() < deadline:
            await asyncio.sleep(0.1)
        if self.alt_m is None:
            raise RuntimeError('No altitude telemetry received from PX4.')

        if cfg.start_with_rtl:
            self.get_logger().info('Triggering RTL...')
            await drone.action.return_to_launch()
        else:
            self.get_logger().info('start_with_rtl is false: waiting for the marker where the drone is.')

    # ----------------------------------------------------------------------
    def _perceive(self, now: float) -> Perception:
        """Turn the newest raw reading into a smoothed, time-checked Perception."""
        cfg = self.cfg
        with self._lock:
            det = self._latest
        age = (now - det.t) if det is not None else math.inf

        if age >= cfg.detection_timeout_s:
            self._streak_start = None
            self._fx = self._fy = None
            return Perception(False, 0.0, 0.0, 0.0)

        if self._streak_start is None:
            self._streak_start = now
        if det.t != self._last_det_t:                 # a NEW reading arrived
            self._last_det_t = det.t
            if self._fx is None:
                self._fx, self._fy = det.nx, det.ny
            else:
                a = cfg.filter_alpha
                self._fx = a * det.nx + (1 - a) * self._fx
                self._fy = a * det.ny + (1 - a) * self._fy
        return Perception(True, now - self._streak_start, self._fx, self._fy)

    def _errors_m(self, nx: float, ny: float, height: float):
        """Convert normalised image errors into metres on the ground.
        At height h the camera sees a strip h*tan(hfov/2) wide on each side of centre."""
        cfg = self.cfg
        scale = max(height, cfg.min_height_for_scale_m) * math.tan(cfg.camera_hfov_rad / 2.0)
        return nx * scale, ny * scale

    def _lateral_cmd(self, ex: float, ey: float):
        """P-controller: speed proportional to distance, capped at a maximum."""
        cfg = self.cfg
        vx = cfg.sign_forward * cfg.kp * ey      # body forward
        vy = cfg.sign_right * cfg.kp * ex        # body right
        speed = math.hypot(vx, vy)
        if speed > cfg.max_horizontal_speed_mps:
            k = cfg.max_horizontal_speed_mps / speed
            vx, vy = vx * k, vy * k
        return vx, vy

    def _descent_speed(self, height: float) -> float:
        cfg = self.cfg
        return _clip(cfg.descent_gain * height, cfg.descent_min_mps, cfg.descent_max_mps)

    # ----------------------------------------------------------------------
    def _fly(self, p: Perception, h: float, alt_ok: bool, now: float):
        """Decide the next velocity command (vx, vy, vz). May change self.state."""
        cfg = self.cfg

        # --- marker just disappeared during ALIGN / DESCEND -> LOST ------------
        if self.state in (State.ALIGN, State.DESCEND) and not p.fresh:
            self._lost_since = now
            self._goto(State.LOST)

        # --- LOST: hover, then climb slowly, then give up ----------------------
        if self.state is State.LOST:
            if p.fresh and p.streak_s >= cfg.reacquire_s:
                self._aligned_since = None
                self._goto(State.ALIGN)
            else:
                lost_for = now - self._lost_since
                if lost_for > cfg.lost_abort_s:
                    self._goto(State.ABORT)
                    return 0.0, 0.0, 0.0
                vz = 0.0
                if lost_for > cfg.lost_hover_s and alt_ok and h < cfg.max_recover_alt_m:
                    vz = -cfg.recover_climb_mps          # negative = up
                self._status(f'[LOST] marker missing for {lost_for:.1f} s, height {h:.1f} m')
                return 0.0, 0.0, vz

        ex, ey = self._errors_m(p.nx, p.ny, h)
        err = math.hypot(ex, ey)
        vx, vy = self._lateral_cmd(ex, ey) if p.fresh else (0.0, 0.0)
        vz = 0.0

        if self.state is State.ALIGN:
            if err < cfg.align_threshold_m:
                if self._aligned_since is None:
                    self._aligned_since = now
                elif now - self._aligned_since >= cfg.align_dwell_s:
                    self._goto(State.DESCEND)
            else:
                self._aligned_since = None
            self._status(f'[ALIGN] err={err:.2f} m  height={h:.1f} m')

        elif self.state is State.DESCEND:
            if err > cfg.realign_threshold_m:
                self._aligned_since = None
                self._goto(State.ALIGN)
            elif h <= cfg.final_descent_alt_m:
                self._goto(State.FINAL_DESCENT)
            else:
                # Full speed when centred, slowing to zero as the error grows.
                span = max(cfg.realign_threshold_m - cfg.align_threshold_m, 1e-3)
                factor = _clip((cfg.realign_threshold_m - err) / span, 0.0, 1.0)
                vz = self._descent_speed(h) * factor if alt_ok else 0.0
            self._status(f'[DESCEND] err={err:.2f} m  height={h:.1f} m  vz={vz:.2f}')

        elif self.state is State.FINAL_DESCENT:
            # Very close to the ground the marker can leave the picture. We keep
            # descending straight down (vx = vy = 0 if the marker is not seen).
            vz = cfg.descent_min_mps if alt_ok else 0.0
            if alt_ok and h <= cfg.land_altitude_m:
                self._goto(State.LAND)
            self._status(f'[FINAL] height={h:.2f} m')

        return vx, vy, vz

    # ----------------------------------------------------------------------
    async def _send(self, drone: System, vx: float, vy: float, vz: float, dt: float) -> None:
        """Send a velocity command, limiting how fast it may change (smooth motion).
        Yaw rate is always 0: the drone keeps its heading (see CODE_AUDIT.md)."""
        cfg = self.cfg
        cx, cy, cz = self._cmd
        dh = cfg.max_horizontal_accel_mps2 * dt
        dv = cfg.max_vertical_accel_mps2 * dt
        vx, vy, vz = _step_towards(cx, vx, dh), _step_towards(cy, vy, dh), _step_towards(cz, vz, dv)
        self._cmd = (vx, vy, vz)
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))

    async def _start_offboard(self, drone: System, dt: float) -> bool:
        cfg = self.cfg
        self.get_logger().info('Pre-streaming zero-velocity setpoints...')
        self._cmd = (0.0, 0.0, 0.0)
        for _ in range(int(cfg.prestream_s / dt)):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(dt)
        try:
            await drone.offboard.start()
            self.get_logger().info('OFFBOARD started')
            return True
        except OffboardError as exc:
            self.get_logger().error(f'OFFBOARD failed: {exc}')
            return False

    async def _do_land(self, drone: System) -> None:
        cfg = self.cfg
        self.get_logger().info('Handing over to PX4 LAND')
        await drone.action.land()
        deadline = time.monotonic() + cfg.land_timeout_s
        while time.monotonic() < deadline:
            if self.landed_state == LandedState.ON_GROUND:
                self.get_logger().info('Touchdown confirmed')
                return
            await asyncio.sleep(0.5)
        self.get_logger().warn('Did not confirm touchdown before timeout (PX4 is still in LAND mode)')

    async def _abort(self, drone: System) -> None:
        """Safe fallback: stop OFFBOARD and ask PX4 to hold position."""
        self.get_logger().error('ABORT: stopping OFFBOARD and holding position')
        try:
            await drone.offboard.stop()
        except Exception:
            pass
        try:
            await drone.action.hold()
        except Exception as exc:
            self.get_logger().error(f'Hold command failed: {exc}')

    # ----------------------------------------------------------------------
    async def _control_loop(self, drone: System) -> None:
        cfg = self.cfg
        dt = 1.0 / cfg.control_hz
        next_tick = time.monotonic()

        while rclpy.ok() and self.state not in (State.DONE, State.ABORT):
            now = time.monotonic()

            if self._stop_requested.is_set():
                self.get_logger().warn('Stop requested (Ctrl+C)')
                self._goto(State.ABORT)
                break

            p = self._perceive(now)
            h = self.alt_m if self.alt_m is not None else 0.0
            alt_ok = self.alt_m is not None and (now - self.alt_time) < cfg.telemetry_timeout_s

            if self.state is State.WAIT_FOR_MARKER:
                self._status(f'[WAIT] waiting for marker ({p.streak_s:.1f}/{cfg.stable_detect_s:.0f} s)')
                if p.fresh and p.streak_s >= cfg.stable_detect_s:
                    self._goto(State.OFFBOARD_INIT)

            elif self.state is State.OFFBOARD_INIT:
                ok = await self._start_offboard(drone, dt)
                self._goto(State.ALIGN if ok else State.ABORT)

            elif self.state is State.LAND:
                await self._do_land(drone)
                self._goto(State.DONE)

            else:   # ALIGN, DESCEND, LOST, FINAL_DESCENT
                vx, vy, vz = self._fly(p, h, alt_ok, now)
                if self.state not in (State.ABORT, State.LAND):
                    await self._send(drone, vx, vy, vz, dt)

            next_tick += dt
            await asyncio.sleep(max(0.0, next_tick - time.monotonic()))


# ---------------------------------------------------------------------------
def main() -> None:
    rclpy.init()
    node = PrecisionLanding()
    try:
        while rclpy.ok() and not node.finished.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.request_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
