"""Closed-loop scripted demo for burette-titration, played by ``yam_scripted`` (see ``titrkit._patch_scripted``).

It sees only what the robot sees: the top camera's RGB image and the arms' measured joint positions. It knows the
layout from the setup (where the stand, the key and the flask are), not the equivalence volume and not the
simulator's state.

1. The right arm reaches in from the right, grips the flask's neck below the rim (fingers either side of it, the tip
   inside the neck untouched) and lifts it 4 mm off the tile. The left arm reaches the stopcock key along its axis
   and grips its handle wings between the pads (fingers closing vertically: the wings lie across the burette).
2. The right arm swirls the flask for the rest of the titration: the neck goes round 4 mm circles twice a second
   (the tip has 7.5 mm of clearance in the neck).
3. **Coarse.** The left wrist rolls the key 86 degrees from closed (a fast drip, about 5 drops a second). The top
   camera watches the flask's liquid: each drop leaves a pink patch that the swirling clears; near the end point
   the patches linger and grow. When the patch stays large (:func:`pink_fraction` over ``LINGER``), it rolls the
   key back to 79 degrees (about 2 drops a second).
4. **Fine.** It keeps swirling and dripping until the whole liquid has turned pink (``WHOLE``), then rolls the key
   back to closed at once. If a patch lingers very large before that (``SINGLE``), it closes the key and adds
   single drops: open to 77 degrees for 1 s (about one drop), closed, swirl until the patch has faded or the pink
   holds.
5. It swirls 2 s more, sets the flask back down on the tile, lets go of the flask and the key, and backs both arms
   away.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task as T  # noqa: E402

ARMS = ("left", "right")
COARSE, FINE, SINGLE_DEG = 86.0, 79.0, 77.0
SWIRL_R, SWIRL_HZ, LIFT = 0.004, 2.0, 0.004
NECK_Z = T.RIM - 0.016  # TCP on the neck, 1.6 cm below the rim (the tip is 1 cm inside)
LINGER, SINGLE, WHOLE = 0.10, 0.35, 0.30  # strong-pink fraction (flash) / faint-pink fraction (whole liquid)
STRONG_RG = 70
TURN_RATE = 60.0  # deg/s of key roll


def rot_from(approach, close) -> np.ndarray:
    """TCP rotation with its approach axis (z) along ``approach`` and its fingers closing along ``close`` (x: the
    finger tips sit at +-x of the gripper link)."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.c_[x, np.cross(z, x), z]


A = np.asarray(T.KEY_DIR, float)
COCK = np.array([*T.BUR_XY, T.COCK_Z])
KEY_GRIP = COCK - A * 0.026
KEY_R0 = rot_from(A, (0.0, 0.0, 1.0))
NECK = np.array([*T.BUR_XY, NECK_Z])
NECK_R = rot_from((0.0, 1.0, 0.0), (1.0, 0.0, 0.0))


def key_rot(deg: float) -> np.ndarray:
    return axis_angle_matrix(A, math.radians(deg)) @ KEY_R0


# -- vision ----------------------------------------------------------------------------------------------------------


class TopCamera:
    def __init__(self, width, height):
        self.width, self.height = width, height
        self.cam = station.top_camera("calibrated")

    def project(self, p):
        lens = self.cam.lens
        c = quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)).T @ (np.asarray(p, float) - self.cam.pos)
        uf, vf = lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]
        su, sv = lens.width / self.width, lens.height / self.height
        return (uf + 0.5) / su - 0.5, (vf + 0.5) / sv - 0.5


def liquid_box(cam: TopCamera):
    """Image rows/columns spanning the flask's liquid (its layout position)."""
    pts = [cam.project((T.BUR_XY[0] + dx, T.BUR_XY[1] + dy, T.FLOOR + dz))
           for dx in (-0.031, 0.031) for dy in (-0.031, 0.031) for dz in (0.002, 0.014)]
    u = [p[0] for p in pts]
    v = [p[1] for p in pts]
    return (slice(max(0, int(min(v))), int(max(v)) + 1), slice(max(0, int(min(u))), int(max(u)) + 1))


def pink_fraction(img: np.ndarray, box) -> tuple[float, float, int]:
    """Of the pale pixels in the flask's liquid box (the liquid and the white tile behind it): the fraction that are
    strong pink (a fresh flash where the drops land: red far above green) and the fraction that are faint pink (the
    whole liquid at the end point: red a little above green, blue above green). Returns (strong, faint, pale
    pixel count)."""
    roi = img[box].astype(np.float32).reshape(-1, 3)
    r, g, b = roi.T
    pale = (r > 120) & (r + g + b > 300)
    strong = pale & (r - g > STRONG_RG)
    faint = pale & (r - g > 15) & (r - g <= STRONG_RG) & (b - g > 5)
    n = int(pale.sum())
    return (float(strong.sum()) / n if n else 0.0), (float(faint.sum()) / n if n else 0.0), n


# -- control ---------------------------------------------------------------------------------------------------------


class Controller:
    def __init__(self, bases, dt):
        self.dt = dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.grip = dict.fromkeys(ARMS, 1.0)  # 1 open .. 0 closed
        self.start = {a: ik.tcp_pose(self.q[a], self.bases[a]) for a in ARMS}
        self.cam = self.box = None
        self.finished_at = None
        self.phase, self.t_phase = "reach", 0.0
        self.key_deg = 0.0  # commanded
        self.key_goal = 0.0
        self.swirl_amp = 0.0
        self.swirl_on = False
        self.flask_z = 0.0  # lift
        self.hist: list = []
        self.whole_hist: list = []
        self.cheat = os.environ.get("DEMO_CHEAT", "")  # "noswirl", "fullopen"
        self.log_t = -1.0

    def say(self, msg):
        print(f"[titrdemo] {msg}", flush=True)

    def _ik(self, arm, pos, rot):
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q[arm], base_pos=self.bases[arm], iters=60)
        self.q[arm] = q
        return q

    @staticmethod
    def _blend(p0, r0, p1, r1, s):
        s = ik.smoothstep(s)
        return p0 + s * (np.asarray(p1) - p0), ik.slerp_matrix(r0, r1, s)

    def _go(self, name, t):
        self.phase, self.t_phase = name, t
        self.say(f"t {t:5.1f} -> {name}")

    def act(self, obs):
        t = float(obs.state_time)
        img = obs.images["top_cam"]
        if self.cam is None:
            h, w = img.shape[:2]
            self.cam = TopCamera(w, h)
            self.box = liquid_box(self.cam)
        frac, whole, n = pink_fraction(img, self.box)
        self.hist.append(frac)
        self.whole_hist.append(whole)
        tp = t - self.t_phase
        ph = self.phase
        L0p, L0r = self.start["left"]
        R0p, R0r = self.start["right"]
        key_pre = KEY_GRIP - A * 0.05
        neck_pre = NECK + np.array([0.0, -0.07, 0.0])
        left = right = None
        if ph == "reach":  # both arms to their pre-grasp points
            s = min(1.0, tp / 2.5)
            left = self._blend(L0p, L0r, key_pre, KEY_R0, s)
            right = self._blend(R0p, R0r, neck_pre + [0, 0, 0.03], NECK_R, s)
            if tp >= 2.7:
                self._go("approach", t)
        elif ph == "approach":
            s = min(1.0, tp / 1.5)
            left = self._blend(key_pre, KEY_R0, KEY_GRIP, KEY_R0, s)
            right = self._blend(neck_pre + [0, 0, 0.03], NECK_R, neck_pre, NECK_R, min(1.0, 2 * s)) if s < 0.5 else \
                self._blend(neck_pre, NECK_R, NECK, NECK_R, 2 * s - 1)
            if tp >= 1.7:
                self._go("close", t)
        elif ph == "close":
            self.grip = {a: max(0.0, 1.0 - tp / 0.6) for a in ARMS}
            left, right = (KEY_GRIP, KEY_R0), (NECK, NECK_R)
            if tp >= 0.8:
                self._go("lift", t)
        elif ph == "lift":
            self.flask_z = LIFT * min(1.0, tp / 0.6)
            if tp >= 0.8:
                self.swirl_on = self.cheat != "noswirl"
                self._go("coarse", t)
                self.key_goal = 90.0 if self.cheat == "fullopen" else COARSE
        elif ph == "coarse":
            if self.cheat == "fullopen":  # contrast: full stream, shut on the first whole-liquid pink
                if self._whole(whole):
                    self.key_goal = 0.0
                    self._go("settle", t)
            elif tp > 3.0 and frac >= LINGER and self._steady(frac):
                self.key_goal = FINE
                self._go("fine", t)
        elif ph == "fine":
            if self._whole(whole):
                self.key_goal = 0.0
                self._go("settle", t)
            elif tp > 2.0 and frac >= SINGLE and self._steady(frac):
                self.key_goal = 0.0
                self._go("wait", t)
        elif ph == "wait":  # stopcock closed, swirling: does the pink hold or fade?
            if tp > 1.0 and self._whole(whole):
                self._go("settle", t)
            elif tp > 1.0 and frac < LINGER:
                self.key_goal = SINGLE_DEG
                self._go("drop", t)
        elif ph == "drop":
            if tp >= 1.0:
                self.key_goal = 0.0
                self._go("wait", t)
        elif ph == "settle":  # closed; keep swirling 2 s, then put the flask down
            if tp > 2.0:
                self.swirl_on = False
            self.flask_z = LIFT * max(0.0, 1.0 - max(0.0, tp - 2.5) / 0.6)
            if tp >= 3.5:
                self._go("release", t)
        elif ph == "release":
            self.grip = {a: min(1.0, tp / 0.6) for a in ARMS}
            if tp >= 0.8:
                self._go("retreat", t)
        elif ph == "retreat":
            s = min(1.0, tp / 1.5)
            left = self._blend(KEY_GRIP, key_rot(self.key_deg), key_pre, key_rot(self.key_deg), s)
            right = self._blend(NECK, NECK_R, neck_pre, NECK_R, s)
            if tp >= 1.6 and self.finished_at is None:
                self.finished_at = t

        # key roll toward its goal at TURN_RATE
        step = TURN_RATE * self.dt
        self.key_deg += float(np.clip(self.key_goal - self.key_deg, -step, step))
        if left is None:
            left = (KEY_GRIP, key_rot(self.key_deg))
        if right is None:
            self.swirl_amp += float(np.clip((SWIRL_R if self.swirl_on else 0.0) - self.swirl_amp, -0.004 * self.dt,
                                            0.004 * self.dt))
            w = 2 * math.pi * SWIRL_HZ * t
            right = (NECK + np.array([self.swirl_amp * math.cos(w), self.swirl_amp * math.sin(w), self.flask_z]),
                     NECK_R)
        if t - self.log_t >= 1.0:
            self.log_t = t
            self.say(f"t {t:5.1f} {self.phase:8s} key {self.key_deg:5.1f} flash {frac:.2f} whole {whole:.2f} of {n} px")
        out = []
        for arm, (p, r) in (("left", left), ("right", right)):
            q = self._ik(arm, p, r)
            out.append(np.r_[q, self.grip[arm]])
        return out[0], out[1], self.finished_at

    def _whole(self, whole, n=4) -> bool:
        h = self.whole_hist[-n:]
        return len(h) == n and min(h) >= WHOLE

    def _steady(self, frac, n=4) -> bool:
        """The last n frames all agree (not a single bright frame)."""
        h = self.hist[-n:]
        return len(h) == n and min(h) >= 0.9 * frac
