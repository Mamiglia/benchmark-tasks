"""phone_demo: the scripted demo that yam_scripted runs for this task. NumPy only.

It knows the start layout (as every yam_scripted demo does) and reads only the joint positions in each observation.
The left arm brings its open fingers in from the front along the screen's normal, one pad either side of the
phone's width at the height of the volume-up button (above the volume-down button), pinches the two edges at once
(one fast close, about 5 N per pad), lets go 0.3 s later, and backs out. The right arm stays still.

``DEMO_CHEAT`` (environment variable) plays a wrong approach instead, to check the scorer:
``hold`` (the same pinch held 3 s), ``low`` (the pinch at volume-down height), ``lift`` (pinch, then rise 3 cm
before letting go), ``poke`` (push the volume-up button with closed fingertips from the left only).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots_yam.packing import STATE_KEY, pack

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import grip_rotation

OPENING = 2 * abs(FINGER_OPEN)  # m at g = 1
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

# The hand comes in along the screen's normal (the phone leans back 20 degrees, so the approach axis points 70 degrees
# from straight down, towards +x), the fingers closing along world y across the phone's width.
APPROACH_DEG = 70.0
GRIP_R = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -np.radians(APPROACH_DEG)) @ grip_rotation(0.0)
AXIS = GRIP_R[:, 2]  # approach direction (world)
DEBUG = bool(os.environ.get("PHONE_DEBUG"))
PAD_AT = float(os.environ.get("PHONE_PAD_AT", "0.0435"))  # m: the pads' centre this far above the phone's centre
PAD_SPAN = 0.0257  # m: a finger pad's contact face across the approach axis (YAM tip_left collision mesh)
SQUEEZE = 0.0025  # m: command the opening this much below the span with both buttons at their stops


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self.script = None
        self.done = False

    def move(self, to, duration, g=None):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        n = max(1, round(duration / self.ctl.dt))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.pos = start + s * (np.asarray(to, float) - start)
            self.rot = ik.slerp_matrix(r0, GRIP_R, s)
            if g is not None:
                self.g = g0 + s * (g - g0)
            yield

    def wait(self, duration):
        for _ in range(max(1, round(duration / self.ctl.dt))):
            yield

    def measured_g(self):
        return self.ctl.joints[self.side][6]

    def note(self, msg):
        if DEBUG:
            print(f"[phone_demo] t {self.ctl.t:5.2f} {self.side}: {msg}", flush=True)

    def pinch(self, z_p, hold, rise=0.0):
        """Pads centred ``z_p`` above the phone's centre along its length, the tool point at mid-thickness."""
        t, p = self.ctl.task, self.ctl.task.PARAMS
        x, z = t.button_point(z_p)
        at = np.array([x, 0.0, z])
        back = at - 0.07 * AXIS
        span = p.width + 2 * (p.proud - p.travel)
        yield from self.move(back + (0, 0, 0.03), 2.0, g=1.0)
        yield from self.move(back, 0.8)
        yield from self.move(at, 1.5)
        yield from self.wait(0.3)
        self.note(f"at pinch point, measured g {self.measured_g():.3f}")
        yield from self.move(self.pos, 0.2, g=g_of(span - SQUEEZE))
        yield from self.wait(hold)
        if rise:
            yield from self.move(self.pos + (0, 0, rise), 0.6)
        self.note(f"pinched, measured opening {self.measured_g() * OPENING * 1000:.1f} mm")
        yield from self.move(self.pos, 0.2, g=1.0)
        yield from self.wait(0.3)
        yield from self.move(self.pos - 0.07 * AXIS, 1.2)

    def poke(self, z_p):
        """Closed fingertips pushed onto the volume-up button from the left (+y) only."""
        t, p = self.ctl.task, self.ctl.task.PARAMS
        x, z = t.button_point(z_p)
        out = p.width / 2 + 0.03
        yield from self.move((x, out, z) - 0.07 * AXIS + (0, 0, 0.03), 2.0, g=0.0)
        yield from self.move((x, out, z), 1.5)
        yield from self.move((x, p.width / 2 + p.proud - p.travel - 0.003, z), 0.6)
        yield from self.wait(0.3)
        yield from self.move((x, out, z), 0.6)
        yield from self.move(self.pos - 0.07 * AXIS, 1.0)

    def run(self, cheat):
        p = self.ctl.task.PARAMS
        if cheat == "hold":
            yield from self.pinch(PAD_AT, 3.0)
        elif cheat == "low":
            yield from self.pinch(p.button_z(2), 0.3)
        elif cheat == "lift":
            yield from self.pinch(PAD_AT, 0.3, rise=0.03)
        elif cheat == "poke":
            yield from self.poke(p.button_z(1))
        else:
            yield from self.pinch(PAD_AT, 0.3)
        self.done = True
        while True:
            yield

    def action7(self):
        q, _, _ = ik.solve(self.pos, ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=60)
        self.q = q
        return np.r_[q, self.g]


class Idle(Arm):
    def action7(self):
        return np.r_[READY, 1.0]


class Controller:
    def __init__(self, bases, dt, task, cheat=""):
        self.dt, self.task, self.t = float(dt), task, 0.0
        self.joints = {"left": np.r_[READY, 1.0], "right": np.r_[READY, 1.0]}
        self.arms = {"left": Arm("left", bases["left"], self), "right": Idle("right", bases["right"], self)}
        self.arms["left"].script = self.arms["left"].run(cheat)
        self.end_t = None
        self.hold_after = 1.5

    def act(self, observation):
        state = np.asarray(observation.state[STATE_KEY], float).reshape(-1)
        self.joints = {"left": state[:7], "right": state[7:14]}
        next(self.arms["left"].script)
        self.t += self.dt
        if self.arms["left"].done and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + self.hold_after
        return pack(self.arms["left"].action7(), self.arms["right"].action7()), done
