"""wiperdemo: the closed-loop scripted demo that yam_scripted runs for this task. NumPy only.

It reads the simulator state the task's physics hook keeps (arm angle, blade poses, latch states), as scripted demos
may. Right hand: grips the arm's shaft 26 cm from the pivot, lifts the arm 14 degrees off the glass and holds it
there against its spring (servoing the measured arm angle), then lowers it onto the glass and lets go. Left hand:
presses the old blade's release tab and pushes the blade 2 cm toward the pivot (out of the hook's bend), takes the
blade, draws it off the hook and lays it on the table; picks the new blade out of its tray, brings its adapter up
under the hook behind the bend, lined up with the arm, and pulls it toward the bend until the latch clicks.

``DEMO_CHEAT`` plays a loophole instead: ``drop`` (lets go of the arm once the new blade is on, 14 degrees up),
``yank`` (pulls the old blade off without pressing the tab).
"""

from __future__ import annotations

import math
import os

import numpy as np
from inspect_robots_yam.packing import STATE_KEY, pack

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

OPENING = 2 * abs(FINGER_OPEN)
DEBUG = bool(os.environ.get("WIPER_DEBUG"))
TOP = 0.7475
LIFT_DEG = 14.0
HOLD_S = 0.22  # right hand on the shaft, from the pivot (clear of the left hand at the hook)
GRASP = np.array([0.09, 0.0, 0.018])  # left hand on the bridge, 9 cm past the adapter (its wrist camera then clears the arm)
TAB_TOP = np.array([-0.010, 0.024, 0.033])  # release tab's top, blade frame
DROP = np.array([0.30, 0.56, TOP + 0.03])  # old blade laid here (blade origin), running along -x
R_DROP = np.diag([-1.0, -1.0, 1.0])


def rot(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - z * np.dot(close, z)
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def rotvec(r) -> np.ndarray:
    """Axis times angle of a rotation matrix."""
    ang = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return v / (2 * math.sin(ang)) * ang


def expm(w) -> np.ndarray:
    """Rotation matrix of the rotation vector ``w`` (Rodrigues)."""
    ang = float(np.linalg.norm(w))
    if ang < 1e-12:
        return np.eye(3)
    k = np.asarray(w) / ang
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(ang) * kx + (1 - math.cos(ang)) * kx @ kx


def g_of(opening: float) -> float:
    return float(np.clip(opening / OPENING, 0.0, 1.0))


class Arm:
    def __init__(self, side, base, ctl):
        self.side, self.base, self.ctl = side, np.asarray(base, float), ctl
        self.q = np.array(READY, float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.g = 1.0
        self.done = False

    def move(self, to, duration, rot=None, g=None, roll=False):
        start, g0, r0 = self.pos.copy(), self.g, self.rot
        r1 = self.rot if rot is None else (self.pick_roll(rot) if roll else rot)
        n = max(1, round(duration / self.ctl.dt))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.pos = start + s * (np.asarray(to, float) - start)
            self.rot = ik.slerp_matrix(r0, r1, s)
            if g is not None:
                self.g = g0 + s * (g - g0)
            yield

    def pick_roll(self, r):
        """Of the two grips that differ by a half turn about the approach axis, the one nearer the current one."""
        flip = r @ np.diag([-1.0, -1.0, 1.0])
        return r if np.trace(self.rot.T @ r) >= np.trace(self.rot.T @ flip) else flip

    def settle(self, target, duration, gain=0.3):
        """Hold ``target`` for the tool point, correcting the command by the measured error (the arm's sag)."""
        bias = np.zeros(3)
        for _ in range(max(1, round(duration / self.ctl.dt))):
            bias = np.clip(bias + gain * (np.asarray(target) - self.ctl.tcp(self.side)), -0.03, 0.03)
            self.pos = np.asarray(target) + bias
            yield

    def wait(self, duration):
        for _ in range(max(1, round(duration / self.ctl.dt))):
            yield

    def until(self, cond, timeout=8.0):
        n = 0
        while not cond() and n * self.ctl.dt < timeout:
            n += 1
            yield

    def note(self, msg):
        if DEBUG:
            print(f"[wdemo] t {self.ctl.t:6.2f} {self.side}: {msg}", flush=True)

    def action7(self):
        """IK for the commanded pose. The warm start is the last solution; if it is stuck (pose or rotation off), seeds
        from the ready pose and random ones are tried, and the joints slew to the new solution at 1.2 rad/s."""
        target = ik.matrix_to_quat_xyzw(self.rot)
        warm = getattr(self, "q_sol", self.q)
        q, e, er = ik.solve(self.pos, target, warm, base_pos=self.base, iters=60)
        if e + 0.05 * er > 0.002:
            seeds = [np.array(READY, float)] + [warm + self.ctl.rng.normal(0.0, 0.5, 6) for _ in range(4)] + \
                    [np.array(READY, float) + self.ctl.rng.normal(0.0, 0.8, 6) for _ in range(4)]
            for seed in seeds:
                q2, e2, er2 = ik.solve(self.pos, target, seed, base_pos=self.base, iters=150)
                if e2 + 0.05 * er2 < e + 0.05 * er - 0.001:
                    q, e, er = q2, e2, er2
        self.q_sol, self.ik_err = q, (e, er)
        self.q = self.q + np.clip(q - self.q, -0.06, 0.06)
        return np.r_[self.q, self.g]


class Controller:
    def __init__(self, bases, dt, sim, layout, cheat=""):
        self.dt, self.t, self.cheat, self.sim, self.L = float(dt), 0.0, cheat, sim, layout
        self.flags: dict = {}
        self.rng = np.random.default_rng(0)
        self.left = Arm("left", bases["left"], self)
        self.right = Arm("right", bases["right"], self)
        self.left.script = self.left_script()
        self.right.script = self.right_script()
        self.end_t = None

    # -- simulator state --
    def theta(self) -> float:
        return float(self.sim.get("theta", 0.0))

    def blade(self, n):
        return self.sim["states"][n]

    def state(self, n) -> str:
        return self.sim.get("mon", {}).get(n, "")

    def rel(self, n):
        return self.sim["rel"][n]

    def tcp(self, side) -> np.ndarray:
        return np.asarray(self.sim["hands"][side][0], float)

    # -- right hand: hold the arm --
    def hold_pose(self, theta):
        ra = self.L.arm_rot(theta)
        return self.L.arm_point(theta, (0.0, HOLD_S, 0.0)), ra @ rot((0, 0, -1), (1, 0, 0))

    def right_script(self):
        a = self.right
        p0, r0 = self.hold_pose(0.0)
        n0 = self.L.arm_rot(0.0)[:, 2]
        yield from a.move(p0 + 0.07 * n0, 2.5, rot=r0, g=g_of(0.035), roll=True)
        yield from a.move(p0 + 0.002 * n0, 1.2)
        yield from a.move(a.pos, 0.6, g=g_of(0.005))
        cmd = 0.0  # commanded arm angle; the measured one lags under the spring
        goal = [0.0]

        def servo():
            nonlocal cmd
            cmd += 0.15 * (goal[0] - self.theta())
            cmd = float(np.clip(cmd, goal[0] - math.radians(6), goal[0] + math.radians(8)))
            a.pos, a.rot = self.hold_pose(cmd)
            a.rot = a.pick_roll(a.rot)

        for k in range(1, 61):  # lift over 3 s
            goal[0] = math.radians(LIFT_DEG) * ik.smoothstep(k / 60)
            cmd = max(cmd, goal[0])
            servo()
            yield
        while abs(math.degrees(self.theta()) - LIFT_DEG) > 0.7:
            servo()
            yield
        a.note(f"arm up {math.degrees(self.theta()):.1f} deg (command {math.degrees(cmd):.1f})")
        self.flags["lifted"] = True
        while not self.flags.get("installed"):
            servo()
            yield
        if self.cheat == "drop":
            a.note("cheat: letting go of the lifted arm")
            yield from a.move(a.pos, 0.3, g=g_of(0.035))
            yield from a.move(a.pos + 0.06 * n0, 1.0)
        else:
            start = goal[0]
            for k in range(1, 81):  # lower over 4 s
                goal[0] = start * (1 - ik.smoothstep(k / 80))
                servo()
                yield
            goal[0] = math.radians(-0.5)
            for _ in range(20):
                servo()
                yield
            a.note(f"arm down {math.degrees(self.theta()):.2f} deg")
            yield from a.move(a.pos, 0.6, g=g_of(0.035))
            yield from a.move(a.pos + 0.07 * n0, 1.2)
        yield from a.move(np.array([0.20, -0.25, TOP + 0.25]), 1.5)
        a.done = True
        while True:
            yield

    # -- left hand --
    def blade_grip(self, o, r):
        """Tool point and rotation for the top grasp of a blade at pose (o, r)."""
        return o + r @ GRASP, r @ rot((0, 0, -1), (0, 1, 0))  # the roll the left arm reaches at every blade pose

    def servo_blade(self, n, o_goal, r_goal, duration, bias=None, gain=0.25):
        """Move the held blade's origin to ``o_goal`` and its axes to ``r_goal`` (world), correcting the hand's
        command for the grip offset, slip and the arm's sag (integrated position and rotation errors)."""
        a = self.left
        b = self.blade(n)
        o0 = b.pos.copy()
        bias, rc = (np.zeros(3), np.eye(3)) if bias is None else bias
        steps = max(1, round(duration / self.dt))
        for k in range(1, steps + 1):
            s = ik.smoothstep(k / steps)
            o_now = o0 + s * (o_goal - o0)
            b = self.blade(n)
            bias = np.clip(bias + gain * (o_now - b.pos), -0.06, 0.06)
            w = rotvec(r_goal @ b.rotation().T)
            rc = expm(0.1 * w) @ rc
            if np.linalg.norm(rotvec(rc)) > math.radians(20):
                rc = expm(rotvec(rc) * math.radians(20) / np.linalg.norm(rotvec(rc)))
            p, r = self.blade_grip(o_now, r_goal)
            a.pos, a.rot = p + bias, rc @ r
            yield
        return bias, rc

    def left_script(self):
        a = self.left
        yield from a.until(lambda: self.flags.get("lifted"), timeout=30.0)
        # 1. press the old blade's tab and push the blade 2 cm toward the pivot
        b = self.blade("old_blade")
        R = b.rotation()
        bx, by, bz = R[:, 0], R[:, 1], R[:, 2]
        ap = math.cos(math.radians(20)) * -bz + math.sin(math.radians(20)) * -by
        ap /= np.linalg.norm(ap)
        top = b.to_world(TAB_TOP)
        r_tab = rot(ap, bx)
        if self.cheat != "yank":
            yield from a.move(top - ap * 0.06, 2.5, rot=r_tab, g=0.0, roll=True)
            yield from a.move(top - ap * 0.014, 1.0)
            yield from a.move(top - ap * (0.013 - 0.006), 0.8)
            yield from a.until(lambda: self.state("old_blade") == "slide", timeout=2.0)
            a.note(f"tab: state {self.state('old_blade')} tab {self.blade('old_blade').slides[0] * 1000:.1f} mm")
            yield from a.move(a.pos - bx * 0.024, 1.5)
            yield from a.wait(0.3)
            a.note(f"slid: rel {np.round(self.rel('old_blade')[0] * 1000, 1)}")
            yield from a.move(a.pos - ap * 0.05, 1.0)
        # 2. take the blade by its bridge
        b = self.blade("old_blade")
        p, r = self.blade_grip(b.pos, b.rotation())
        bz = b.rotation()[:, 2]
        yield from a.move(p + 0.06 * bz, 2.0, rot=r, g=g_of(0.032))
        yield from a.move(p, 1.0)
        yield from a.settle(p, 0.8)
        a.note(f"at the old blade: tool point off by {np.round((self.tcp('left') - p) * 1000, 1)} mm, ik {a.ik_err}")
        yield from a.move(a.pos, 0.6, g=0.0)
        yield from a.until(lambda: self.state("old_blade") == "free", timeout=1.5 if self.cheat != "yank" else 0.1)
        a.note(f"holding old blade: state {self.state('old_blade')}")
        # 3. draw it off the hook (away from the arm, then toward the robot), carry it to the table
        if self.cheat == "yank":
            yield from a.move(a.pos - 0.06 * bz + np.array([-0.05, 0.05, 0.05]), 1.0)
        yield from a.move(a.pos - 0.015 * bz, 0.8)  # adapter below the return leg; the blade end stays off the glass
        yield from a.move(a.pos + np.array([-0.07, 0.0, 0.06]), 1.2)
        yield from a.move(np.array([0.28, 0.40, TOP + 0.26]), 2.0)
        p, r = self.blade_grip(DROP, R_DROP)
        yield from a.move(p + np.array([0.0, 0.0, 0.12]), 2.5, rot=r)
        yield from a.move(p + np.array([0.0, 0.0, 0.0]), 1.5)
        yield from a.move(a.pos, 0.5, g=g_of(0.032))
        yield from a.move(a.pos + np.array([0.0, 0.0, 0.10]), 1.0)
        a.note(f"old blade on the table: state {self.state('old_blade')} z {self.blade('old_blade').low[2] - TOP:.3f}")
        # 4. take the new blade out of its tray
        b = self.blade("new_blade")
        p, r = self.blade_grip(b.pos, b.rotation())
        p = p + np.array([0.0, 0.0, 0.008])  # fingertips clear of the tray walls
        yield from a.move(p + np.array([0.0, 0.0, 0.08]), 2.0, rot=r, g=g_of(0.032))
        yield from a.move(p, 1.2)
        yield from a.settle(p, 0.8)
        yield from a.move(a.pos, 0.6, g=0.0)
        yield from a.move(a.pos + np.array([0.0, 0.0, 0.12]), 1.5)
        # 5. bring the adapter under the hook, behind its bend, lined up with the arm
        th = self.theta()
        so, sr = self.L.seat(th)
        u = sr[:, 1] * -1.0  # up the glass (the blade's -y)
        below = so + sr @ np.array([-0.012, 0.0, -0.013])
        # carry it toward the arms first, then turn it in front of the windshield (not over it), lined up with the arm
        p, r = self.blade_grip(np.array([0.17, 0.42, TOP + 0.15]), R_DROP)
        yield from a.move(p, 2.0, rot=r)
        w1 = below - 0.10 * u + np.array([0.0, 0.0, 0.05])
        p, r = self.blade_grip(w1 + np.array([0.0, 0.28, 0.06]), sr)
        yield from a.move(p, 4.0, rot=r)
        p, r = self.blade_grip(w1, sr)
        yield from a.move(p, 2.0)
        p, r = self.blade_grip(below - 0.10 * u, sr)
        yield from a.move(p, 1.5)
        bias = yield from self.servo_blade("new_blade", below, sr, 3.0)
        th = self.theta()
        so, sr = self.L.seat(th)
        bias = yield from self.servo_blade("new_blade", so + sr @ np.array([-0.012, 0.0, -0.001]), sr, 1.5, bias)
        bias = yield from self.servo_blade("new_blade", so + sr @ np.array([-0.012, 0.0, 0.0]), sr, 0.5, bias)
        a.note(f"under the hook: state {self.state('new_blade')} rel {np.round(self.rel('new_blade')[0] * 1000, 1)} "
               f"ang {self.rel('new_blade')[1]:.1f}")
        # 6. pull it toward the bend until it clicks
        if self.state("new_blade") == "slide":
            for x in (0.0, 0.004, 0.008):
                if self.state("new_blade") == "latched":
                    break
                so, sr = self.L.seat(self.theta())
                bias = yield from self.servo_blade("new_blade", so + sr @ np.array([x, 0.0, 0.0]), sr, 1.0, bias,
                                                   gain=0.15)
        a.note(f"clicked? state {self.state('new_blade')} rel {np.round(self.rel('new_blade')[0] * 1000, 1)}")
        yield from a.move(a.pos, 0.5, g=g_of(0.032))
        bz = self.blade("new_blade").rotation()[:, 2]
        yield from a.move(a.pos + 0.06 * bz, 1.0)
        yield from a.move(np.array([0.22, 0.35, TOP + 0.25]), 1.5)
        self.flags["installed"] = True
        a.done = True
        while True:
            yield

    def act(self, observation):
        if "states" not in self.sim or not self.sim["states"]:
            return pack(np.r_[READY, 1.0], np.r_[READY, 1.0]), False
        next(self.left.script)
        next(self.right.script)
        self.t += self.dt
        if self.left.done and self.right.done and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + 2.0
        return pack(self.left.action7(), self.right.action7()), done
