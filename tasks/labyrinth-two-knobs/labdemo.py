"""Closed-loop scripted demo for labyrinth-two-knobs, played by ``yam_scripted`` (see ``labkit._patch_scripted``).

1. **Grip the knobs.** Each arm brings its gripper level, pointing along its knob's shaft, 7 cm out; moves in along
   the shaft to the knob and closes on it (the right arm on the end knob, ``x``; the left arm on the side knob,
   ``y``). From here on only the wrist roll (joint 6) moves: rolling the wrist turns the knob.
2. **Steer.** Every control step (20 Hz) it reads the ball's position and velocity on the board and both knob
   angles (simulator state; a person reads them by eye), and computes the acceleration that pulls the ball toward a
   point 25 mm ahead of it on its line along the path, at 4 cm/s: a = 3 (target - p) + 2.5 (v_line - v), at most
   0.25 m/s^2. The line follows the drawn path but swings 8 mm away from each hole as the ball passes it. Each tilt
   is the one giving that acceleration (a = 5/7 g sin tilt for a rolling ball), each knob angle 25 times its tilt,
   and each wrist roll is set to turn its knob there, plus an integral term that removes the knob's lag behind the
   wrist (grip slip, the arm's compliance).
3. After FINISH it levels the board, opens the hands and backs off.

``DEMO_CHEAT``: ``none`` grips the knobs and then holds them still; ``onearm`` steers with the end knob only (the
other arm never grips).
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

import labkit
import labmodel as M
import task

ARMS = ("left", "right")
KNOB_OF = {"left": "y", "right": "x"}
GRIP = 0.30  # normalised finger command on the knob: pads 3 mm inside its 35 mm rim (about 15 N squeeze)
PRE = 0.07
LOOK, SPEED, KP, KV, AMAX, AWAY = 0.025, 0.04, 3.0, 2.5, 0.25, 0.008
KI = 2.0  # 1/s, wrist-roll integral on the knob's error
G_ROLL = 9.81 * 5 / 7
INITS = {"left": [-0.34, 1.65, 0.13, 1.52, 0.45, 0.0], "right": [0.28, 1.58, 0.05, 1.53, -0.51, 0.0]}


def grip_rot(out) -> np.ndarray:
    """Gripper level, approach (z) along -out, fingers closing horizontally, gripper y down (camera up)."""
    z = -np.asarray(out, float)
    x = np.array([-z[1], z[0], 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.c_[x, y, z]
    return R if R[2, 1] < 0 else R @ np.diag([-1.0, -1.0, 1.0])


def demo_line(n: int = 600) -> np.ndarray:
    """The ball's planned line (board frame): the drawn path, swung AWAY from each hole near it."""
    pts = []
    for s in np.linspace(0.0, M.PATH.length, n):
        p = M.PATH.point(s)
        q = M.PATH.point(min(s + 1e-3, M.PATH.length)) - M.PATH.point(max(s - 1e-3, 0.0))
        nrm = np.array([-q[1], q[0]]) / (np.linalg.norm(q) + 1e-12)
        off = np.zeros(2)
        for h, hs in zip(M.HOLES, M.HOLE_S):
            w = max(0.0, math.cos(min(abs(s - hs) / 0.03, 1.0) * math.pi / 2))
            side = np.sign((h - p) @ nrm)
            off -= side * nrm * AWAY * w
        pts.append(p + off)
    return np.array(pts)


class Controller:
    def __init__(self, bases, dt):
        self.dt = dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.cheat = os.environ.get("DEMO_CHEAT", "")
        self.line = demo_line()
        self.line_s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.line, axis=0), axis=1))]
        self.plan = {a: self._approach(a) for a in ARMS}
        self.q_grip = {a: self.plan[a][-1].copy() for a in ARMS}
        self.t_grip = None
        self.theta0 = {}
        self.integ = dict.fromkeys(ARMS, 0.0)
        self.k = 0
        self.finished_at = None
        self.t_done = None
        self.say(f"line {self.line_s[-1]:.3f} m, {M.N_HOLES} holes")

    def say(self, msg):
        print(f"[labdemo] {msg}", flush=True)

    def _approach(self, arm):
        """Joint waypoints: READY -> 7 cm out on the knob's axis -> at the grip point (straight in, 8 steps)."""
        p, out = task.knob_grip(KNOB_OF[arm])
        R = grip_rot(out)
        quat = ik.matrix_to_quat_xyzw(R)
        q, pe, re = ik.solve(p + PRE * np.asarray(out), quat, q_init=INITS[arm], base_pos=self.bases[arm], iters=300)
        qs = [q]
        for f in np.linspace(1, 0, 9)[1:]:
            q, pe, re = ik.solve(p + f * PRE * np.asarray(out), quat, q_init=q, base_pos=self.bases[arm], iters=150)
            qs.append(q)
        self.say(f"{arm}: knob {KNOB_OF[arm]} grip at {np.round(p, 3)}, IK {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        return qs

    # -- the ball controller
    def _accel(self, snap):
        p, v = snap.ball_uv, snap.ball_vel_uv
        i = int(np.argmin(np.linalg.norm(self.line - p, axis=1)))
        s = self.line_s[i]
        j = int(np.searchsorted(self.line_s, s + LOOK))
        j = min(j, len(self.line) - 1)
        tgt = self.line[j]
        tan = self.line[min(j + 1, len(self.line) - 1)] - self.line[max(j - 1, 0)]
        tan = tan / (np.linalg.norm(tan) + 1e-12)
        speed = SPEED if s < self.line_s[-1] - 0.03 else 0.0
        a = KP * (tgt - p) + KV * (speed * tan - v)
        n = np.linalg.norm(a)
        return a * (AMAX / n) if n > AMAX else a

    def act(self, obs):
        t = float(obs.state_time)
        snap = labkit.SNAP
        out = {}
        T_PRE, T_IN, T_CLOSE = 2.5, 1.5, 0.8
        for arm in ARMS:
            qs = self.plan[arm]
            grip = 1.0
            if t < T_PRE:
                f = ik.smoothstep(t / T_PRE)
                q = np.asarray(READY) + f * (qs[0] - np.asarray(READY))
            elif t < T_PRE + T_IN:
                f = (t - T_PRE) / T_IN * (len(qs) - 1)
                i = min(int(f), len(qs) - 2)
                q = qs[i] + (f - i) * (qs[i + 1] - qs[i])
            else:
                q = qs[-1].copy()
                grip = max(GRIP, 1.0 - (t - T_PRE - T_IN) / T_CLOSE * (1.0 - GRIP))
            out[arm] = (q, grip)
        if self.cheat == "onearm":
            out["left"] = (np.asarray(READY, float), 1.0)
        if t >= T_PRE + T_IN + T_CLOSE + 0.3:
            if self.t_grip is None:
                self.t_grip = t
                self.theta0 = dict(snap.knob)
                self.say(f"gripped at {t:.2f} s, knobs {dict((k, round(math.degrees(v), 1)) for k, v in snap.knob.items())}")
            done = snap.fell is not None or self.t_done is not None
            a = np.zeros(2) if done or self.cheat == "none" else self._accel(snap)
            tilt = {"y": math.asin(float(np.clip(a[0] / G_ROLL, -1, 1))),
                    "x": -math.asin(float(np.clip(a[1] / G_ROLL, -1, 1)))}
            for arm in ARMS:
                if self.cheat == "onearm" and arm == "left":
                    continue
                k = KNOB_OF[arm]
                want = M.RATIO * float(np.clip(tilt[k], -M.TILT_MAX, M.TILT_MAX))
                err = want - snap.knob[k]
                self.integ[arm] = float(np.clip(self.integ[arm] + KI * err * self.dt, -0.6, 0.6))
                q = self.q_grip[arm].copy()
                q[5] -= (want - self.theta0[k]) + self.integ[arm]  # wrist roll +d turns the knob -d (outward sense)
                out[arm] = (q, GRIP)
            self.k += 1
            if self.k % 10 == 0:
                self.say(f"t {t:6.2f} ball {np.round(snap.ball_uv * 1000, 1)} v {np.round(snap.ball_vel_uv * 1000)} "
                         f"a {np.round(a, 3)} knobs {dict((k, round(math.degrees(v), 1)) for k, v in snap.knob.items())}")
            if self.t_done is None and (snap.fell is not None or
                                        M.PATH.project(snap.ball_uv)[0] > M.HOLE_S[-1] + 0.01):
                self.t_done = t
                self.say(f"{'fell in hole %d' % snap.fell if snap.fell else 'at FINISH'} at {t:.2f} s")
            if self.t_done is not None and t > self.t_done + 1.0:
                for arm in ARMS:
                    q = self.q_grip[arm].copy()
                    q[5] = out[arm][0][5]
                    out[arm] = (q, 1.0)
                if t > self.t_done + 2.0 and self.finished_at is None:
                    self.finished_at = t
        return (np.r_[out["left"][0], out["left"][1]], np.r_[out["right"][0], out["right"][1]], self.finished_at)
