"""feed_demo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It sees what the robot observes:
- the top camera image: a small window round the nest's pocket, as projected from the station's calibrated top
  camera. A bare aluminium blank standing in the black pocket makes that window bright; an empty pocket (or the other
  arm's black gripper above it) leaves it dark.
- the 14-D joint positions: the other arm's six joints, through the URDF forward kinematics, give where its gripper is.
It knows the start layout of the tray, the nest and the chute, as every yam_scripted demo does. It never knows the
other arm's timing or reads any simulator state.

The loop, per blank: take the next blank from the tray, carry it to a standby point 15 cm to the right of the nest,
wait there until the nest has looked empty for 0.3 s AND the other arm's gripper is more than 15 cm from the nest,
then set the blank down in the pocket, let go and leave. While it sets the blank down it steers the MEASURED tool point
(forward kinematics of its own joint readings) onto the pocket: the compliant arm otherwise lands up to 2 cm off. While it sets the blank down it steers the MEASURED tool point
(forward kinematics of its own joint readings) onto the pocket: the compliant arm otherwise lands up to 2 cm off.

``DEMO_CHEAT`` (environment variable) plays a loophole instead, to check that the scorer gives it nothing:
``idle`` (never move), ``timed`` (a fixed clock: set a blank down every 13 s, the mean visit interval, without
looking), ``rush`` (look at the nest but not at the other arm: set the blank down as soon as the pocket looks empty).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import STATE_KEY, pack

from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from yam_arena.urdf import quat_xyzw_to_matrix

DEBUG = bool(os.environ.get("CELL_DEBUG"))
DOWN_R = grip_rotation(0.0)
GRIP_DZ = 0.030  # tool point above the blank's bottom when gripping it
TRAVEL_DZ = 0.13  # tool point above the table while carrying
SET_GAP = 0.003  # blank bottom above the pocket floor when let go
ABOVE_WALL = 0.018  # blank bottom above the table while it is lined up over the pocket (walls 12 mm)
STANDBY_DY = -0.15
CLEAR_M = 0.15  # the other arm's gripper must be this far (xy) from the nest
BRIGHT = 120.0  # mean grey level of the nest window (3 x 3 px) above which a blank stands there
EMPTY_FOR_S = 0.3
WINDOW_PX = 1  # half-size of the nest window at 320 x 176


def project(p, width: int, height: int) -> tuple[float, float]:
    """Pixel (u, v) of world point p in the top camera image resized to width x height."""
    cam = station.top_camera()
    r = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
    pc = r.T @ (np.asarray(p, float) - np.asarray(cam.pos))  # OpenGL camera: looks along -z, +y up
    lens = cam.lens.resized(width, height)
    return lens.cx + lens.fx * pc[0] / -pc[2], lens.cy - lens.fy * pc[1] / -pc[2]


class Controller:
    def __init__(self, bases, dt, table_z, nest_xy, supply, tray_z, blank_h, cheat=""):
        self.dt, self.top, self.nest, self.tray_z, self.blank_h = float(dt), table_z, np.asarray(nest_xy), tray_z, blank_h
        self.bases = {k: np.asarray(v, float) for k, v in bases.items()}
        self.supply = list(supply)
        self.cheat = cheat
        self.t = 0.0
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.bases["right"])
        self.g = 1.0
        self.joints = np.r_[READY, 1.0, READY, 1.0]
        self.image = None
        self.empty_since = None
        self.script = self.run()

    # -- what the robot sees --
    def nest_brightness(self) -> float:
        h, w = self.image.shape[:2]
        u, v = project((*self.nest, self.top + self.blank_h), w, h)
        u, v = int(round(u)), int(round(v))
        k = WINDOW_PX
        return float(self.image[max(v - k, 0):v + k + 1, max(u - k, 0):u + k + 1].mean())

    def other_gripper_xy(self) -> np.ndarray:
        return ik.tcp_pose(self.joints[:6], self.bases["left"])[0][:2]

    def other_clear(self) -> bool:
        return float(np.linalg.norm(self.other_gripper_xy() - self.nest)) > CLEAR_M

    # -- motions --
    def move(self, to, duration: float, g: float | None = None):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        n = max(1, round(duration / self.dt))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.pos = start + s * (np.asarray(to, float) - start)
            self.rot = ik.slerp_matrix(r0, DOWN_R, s)
            if g is not None:
                self.g = g0 + s * (g - g0)
            yield

    def wait(self, duration: float):
        for _ in range(max(1, round(duration / self.dt))):
            yield

    def take(self, xy):
        x, y = xy
        yield from self.move((x, y, self.top + TRAVEL_DZ), 1.6, g=1.0)
        yield from self.move((x, y, self.tray_z + GRIP_DZ), 1.0)
        yield from self.move(self.pos, 0.5, g=0.0)
        yield from self.move((x, y, self.top + TRAVEL_DZ), 0.8)
        yield from self.move((self.nest[0], self.nest[1] + STANDBY_DY, self.top + TRAVEL_DZ), 1.4)

    def hand(self) -> np.ndarray:
        """Tool point from the measured joints (the arm sags and lags its command by up to about 1 cm)."""
        return ik.tcp_pose(self.joints[7:13], self.bases["right"])[0]

    def track(self, start, goal, duration: float, settle: float = 0.0, tol: float = 0.001):
        """Drive the MEASURED tool point along a straight line from ``start`` to ``goal`` (an integral correction on
        top of the commanded path), then hold ``goal`` for up to ``settle`` s until it is within ``tol``."""
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        n = max(1, round(duration / self.dt))
        for k in range(1, n + round(settle / self.dt) + 1):
            g = start + ik.smoothstep(min(k / n, 1.0)) * (goal - start)
            err = g - self.hand()
            if k > n and np.linalg.norm(err) < tol:
                return
            self.offset = self.offset + 0.3 * err
            self.pos = g + self.offset
            yield

    def set_down(self):
        nx, ny = self.nest
        over = (nx, ny, self.top + TRAVEL_DZ)
        above = (nx, ny, self.top + ABOVE_WALL + GRIP_DZ)
        low = (nx, ny, self.top + SET_GAP + GRIP_DZ)
        self.offset = np.zeros(3)
        yield from self.track(self.hand(), over, 0.9, settle=0.6)
        yield from self.track(over, above, 0.8, settle=0.8)
        yield from self.track(above, low, 0.6, settle=0.4, tol=0.0015)
        yield from self.wait(0.1)
        yield from self.move(self.pos, 0.4, g=1.0)
        yield from self.move(self.pos + (0.0, 0.0, TRAVEL_DZ - SET_GAP - GRIP_DZ), 0.6)
        yield from self.move((nx, ny + STANDBY_DY, self.top + TRAVEL_DZ), 0.6)

    def nest_empty(self) -> bool:
        if self.nest_brightness() >= BRIGHT:
            self.empty_since = None
            return False
        if self.empty_since is None:
            self.empty_since = self.t
        return self.t - self.empty_since >= EMPTY_FOR_S - 1e-9

    def note(self, msg: str) -> None:
        if DEBUG:
            print(f"[feed_demo] t {self.t:5.2f}: {msg} (nest {self.nest_brightness():.0f}, other at "
                  f"{np.round(self.other_gripper_xy(), 3)})", flush=True)

    def run(self):
        if self.cheat == "idle":
            while True:
                yield
        for i, xy in enumerate(self.supply):
            yield from self.take(xy)
            self.note(f"holding blank {i + 1} at standby")
            if self.cheat == "timed":
                while self.t < 8.0 + 13.0 * i:
                    yield
            else:
                self.empty_since = None
                while not (self.nest_empty() and (self.cheat == "rush" or self.other_clear())):
                    yield
            self.note(f"setting blank {i + 1} down")
            yield from self.set_down()
        yield from self.move((self.nest[0] - 0.08, self.nest[1] - 0.25, self.top + TRAVEL_DZ), 1.5)
        while True:
            yield

    def act(self, observation):
        self.joints = np.asarray(observation.state[STATE_KEY], float).reshape(-1)
        self.image = np.asarray(observation.images["top_cam"])
        next(self.script)
        self.t += self.dt
        q, _, _ = ik.solve(self.pos, ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.bases["right"], iters=60)
        self.q = q
        return pack(np.r_[READY, 1.0], np.r_[q, self.g]), False
