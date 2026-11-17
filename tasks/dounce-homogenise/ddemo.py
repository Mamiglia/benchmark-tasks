"""Scripted demo for dounce-homogenise, played by ``yam_scripted`` (see ``dkit._patch_scripted``).

It knows the layout from the setup (where the rack, the mortar and the pestle's knob are) and nothing from the
simulator: not the sample volume, not the joint.

1. The left arm reaches in from the left and grips the mortar's reservoir (fingers front and back of it); the right
   arm comes down from above and grips the pestle's knob, the gripper pointing straight down the bore's axis.
2. Ten strokes: the right hand draws the knob straight up by ``RISE`` (34 mm, at most 25 mm/s, below the 50 mm/s at
   which a pull would cavitate) and pushes it back down 2 mm past the floor, pausing at the bottom. The head stays
   under the liquid for any sample of 3.0 mL or more (air is drawn in above 38 mm). The left hand holds the mortar
   down in the rack throughout.
3. Both hands let go and back away; the pestle stays in the mortar.

``DEMO_CHEAT`` runs the contrasting attempts: ``nohold`` (the left hand never grips the mortar), ``high``
(fast strokes to 60 mm, the head clearing the liquid), ``short`` (15 mm strokes).
"""

from __future__ import annotations

import os
import pathlib
import sys

import numpy as np
from yam_arena import ik
from yam_arena.constants import READY

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import dkit  # noqa: E402
import dounce as D  # noqa: E402
import task as T  # noqa: E402

ARMS = ("left", "right")
CHEAT = os.environ.get("DEMO_CHEAT", "")
RISE = {"high": 0.060, "short": 0.015}.get(CHEAT, 0.034)
T_UP, T_DOWN, T_DWELL = (0.8, 0.6, 0.2) if CHEAT == "high" else (2.0, 1.5, 0.4)
PUSH = 0.002  # past the floor at the bottom of each stroke
N = 10


def rot_from(approach, close) -> np.ndarray:
    """TCP rotation with its approach axis (z) along ``approach`` and its fingers closing along ``close``."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.c_[x, np.cross(z, x), z]


AXIS = np.array([*T.XY, T.SEAT_Z])
MORTAR = AXIS + [0.0, 0.0, dkit.FLOOR + D.TUBE_L + dkit.RES_H / 2]
MORTAR_R = rot_from((0.0, -1.0, 0.0), (1.0, 0.0, 0.0))
KNOB = AXIS + [0.0, 0.0, dkit.FLOOR + dkit.GRIP_Z]  # head on the floor
KNOB_R = rot_from((0.0, 0.0, -1.0), (1.0, 0.0, 0.0))


class Controller:
    def __init__(self, bases, dt):
        self.dt = dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.grip = dict.fromkeys(ARMS, 1.0)  # 1 open .. 0 closed
        self.start = {a: ik.tcp_pose(self.q[a], self.bases[a]) for a in ARMS}
        self.phase, self.t0 = "reach", 0.0
        self.finished_at = None
        self.n = 0
        self.last = 0.0

    def say(self, msg):
        print(f"[ddemo] {msg}", flush=True)

    def _go(self, name, t):
        self.phase, self.t0 = name, t
        self.say(f"t {t:5.1f} -> {name}")

    def _ik(self, arm, pos, rot):
        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q[arm], base_pos=self.bases[arm], iters=60)
        self.q[arm] = q
        return q

    @staticmethod
    def _blend(p0, r0, p1, r1, s):
        s = ik.smoothstep(s)
        return p0 + s * (np.asarray(p1) - p0), ik.slerp_matrix(r0, r1, s)

    def act(self, obs):
        t = float(obs.state_time)
        tp = t - self.t0
        mor_pre = MORTAR + [0.0, 0.07, 0.0]
        knob_pre = KNOB + [0.0, 0.0, 0.06]
        hold = CHEAT != "nohold"
        left = (MORTAR, MORTAR_R) if hold else (mor_pre, MORTAR_R)
        right = (KNOB, KNOB_R)
        ph = self.phase
        if ph == "reach":
            s = min(1.0, tp / 2.5)
            left = self._blend(*self.start["left"], mor_pre, MORTAR_R, s)
            right = self._blend(*self.start["right"], knob_pre, KNOB_R, s)
            if tp >= 2.7:
                self._go("approach", t)
        elif ph == "approach":
            s = min(1.0, tp / 1.5)
            left = self._blend(mor_pre, MORTAR_R, *left, s)
            right = self._blend(knob_pre, KNOB_R, KNOB, KNOB_R, s)
            if tp >= 1.7:
                self._go("close", t)
        elif ph == "close":
            self.grip = {"left": max(0.0, 1.0 - tp / 0.6) if hold else 1.0, "right": max(0.0, 1.0 - tp / 0.6)}
            if tp >= 0.9:
                self._go("stroke", t)
        elif ph == "stroke":
            period = T_UP + T_DOWN + T_DWELL
            k, u = divmod(tp, period)
            if u < T_UP:
                z = RISE * ik.smoothstep(u / T_UP)
            elif u < T_UP + T_DOWN:
                z = RISE - (RISE + PUSH) * ik.smoothstep((u - T_UP) / T_DOWN)
            else:
                z = -PUSH
            right = (KNOB + [0.0, 0.0, z], KNOB_R)
            if k >= N:
                right = (KNOB + [0.0, 0.0, -PUSH], KNOB_R)
                self._go("release", t)
        elif ph == "release":
            self.grip = dict.fromkeys(ARMS, min(1.0, tp / 0.6))
            if tp >= 0.8:
                self._go("retreat", t)
        elif ph == "retreat":
            s = min(1.0, tp / 1.5)
            left = self._blend(*left, mor_pre, MORTAR_R, s)
            right = self._blend(KNOB, KNOB_R, knob_pre, KNOB_R, s)
            if tp >= 1.6 and self.finished_at is None:
                self.finished_at = t
        if t - self.last >= 2.0:
            self.last = t
            m = dkit.model()
            self.say(f"t {t:5.1f} {self.phase} | z {m.z * 1000:5.1f} strokes {m.strokes} air {m.air} cav {m.cav}")
        out = []
        for arm, (p, r) in (("left", left), ("right", right)):
            out.append(np.r_[self._ik(arm, p, r), self.grip[arm]])
        return out[0], out[1], self.finished_at
