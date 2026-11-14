"""Closed-loop scripted demo for tote-strap-rake, played by ``yam_scripted`` (see ``totekit._patch_scripted``).

It sees only what the robot sees: the top camera's RGB image and the arms' measured joint positions. It never reads
the simulator.

Vision. With both arms parked out of view, :func:`find_rig` takes the blue tote's pixels and the orange strap's
pixels in the top camera image. Through the camera's calibration (``yam_arena.station.top_camera``) each pixel is a
ray; the tote's pixels are put on the plane of its rim, the strap's on the plane 2 cm above the deck. The tote reading
minus its reading at seed shift 0 (:data:`REF_TOTE`) gives how far the tote sits from its reference place. The strap's
end toward the left (+y) is where it is caught: the ring lies a fixed distance beyond the centroid of that end, along
the strap's direction (:data:`RING_PAST_END`).

Motion (``DEMO=safe``, the default). The left hand comes down from the front, pitched 60 degrees, the fingers 44 mm
apart either side of the 25 mm strap, and pinches it 3.6 cm short of the ring's centre. It lifts 11.5 cm straight up,
turning the grip up to 20 degrees on the way, so the ring rises off the tine, carries it over the tote's rim and drops it
inside, so the strap travels with the tote. Then both hands take the tote's end ledges (grips pitched 30 degrees down, fingers above and below the
ledge), lift it 3 cm, draw it toward the arms past the front of the shelf, lower it onto the table, let go and move up
and back.

Contrasts for the README (``DEMO=...``): ``naive`` skips the strap; ``short`` skips the strap and sets the tote down
just in front of the shelf (the smallest carry that lands it on the table); ``yank`` skips the strap, lifts the tote
6 cm before the carry, so the strap pulls up on the ring; ``hold`` frees the strap and puts the tote down but keeps
holding it.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
import totekit as K
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint

ARMS = ("left", "right")
TOP = K.TOP
RIM = K.DECK_TOP + K.TOTE_H  # the tote's rim on the shelf

# -- vision ----------------------------------------------------------------------------------------------------------


def masks(img: np.ndarray):
    rgb = img.astype(np.float32)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    blue = (b > 90) & (b > 1.6 * r) & (b > 1.15 * g)
    orange = (r > 140) & (g > 0.25 * r) & (g < 0.75 * r) & (b < 0.45 * r)
    return blue, orange


@dataclass
class TopCamera:
    """The station's calibrated top camera (OpenGL convention: looks along -z, +y up), for a resized image."""

    width: int
    height: int
    cam: object = field(default_factory=lambda: station.top_camera("calibrated"))

    def rays(self, u, v) -> tuple[np.ndarray, np.ndarray]:
        """(origin, unit directions (n, 3)) of the rays through pixels (u, v) of the resized image."""
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        su, sv = lens.width / self.width, lens.height / self.height
        uf, vf = (np.asarray(u, float) + 0.5) * su - 0.5, (np.asarray(v, float) + 0.5) * sv - 0.5
        d = np.stack([(uf - lens.cx) / lens.fx, -(vf - lens.cy) / lens.fy, -np.ones_like(uf)], axis=-1)
        d = d @ quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)).T
        return np.asarray(self.cam.pos, float), d / np.linalg.norm(d, axis=-1, keepdims=True)

    def on_height(self, u, v, z: float) -> np.ndarray:
        """Where the rays through pixels (u, v) meet the plane at height z: (n, 3)."""
        o, d = self.rays(u, v)
        return o + d * ((z - o[2]) / d[:, 2])[:, None]

    def project(self, p) -> tuple[float, float]:
        """Pixel (u, v) of the resized image where world point ``p`` appears (for tests)."""
        from yam_arena.urdf import quat_xyzw_to_matrix

        lens = self.cam.lens
        c = quat_xyzw_to_matrix(np.asarray(self.cam.rot_xyzw)).T @ (np.asarray(p, float) - self.cam.pos)
        uf, vf = lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]
        su, sv = lens.width / self.width, lens.height / self.height
        return (uf + 0.5) / su - 0.5, (vf + 0.5) / sv - 0.5


STRAP_Z = K.DECK_TOP + 0.02
END_FRAC = 0.2  # the strap's caught end: its pixels in the last 20% of its length, toward +y
RING_PAST_END = 0.05  # the ring's centre lies this far beyond that end's centroid, along the strap (calibrated)


def find_rig(cam: TopCamera, img, min_px: int = 30):
    """(tote reading (x, y) on the rim plane, the ring's centre (x, y), the strap's unit direction toward the ring),
    or None. The ring is found along the strap:
    the strap's pixels on the plane 2 cm above the deck, their main direction, the centroid of their last fifth toward
    the left (+y), and RING_PAST_END beyond it."""
    blue, orange = masks(img)
    vb, ub = np.nonzero(blue)
    vo, uo = np.nonzero(orange)
    if len(ub) < min_px or len(uo) < 6:
        return None
    tote = cam.on_height(ub, vb, RIM)[:, :2].mean(axis=0)
    strap = cam.on_height(uo, vo, STRAP_Z)[:, :2]
    c = strap - strap.mean(axis=0)
    u = np.linalg.svd(c, full_matrices=False)[2][0]  # the strap's direction on the deck
    u = u if u[1] > 0 else -u  # pointing to its caught (left) end
    s = c @ u
    end = strap[s >= s.max() - END_FRAC * (s.max() - s.min())].mean(axis=0)
    return tote, end + RING_PAST_END * u, u


# Readings in the reference image (seed shift 0, both arms parked): see README, "Demo calibration".
REF_TOTE = np.array([0.4338, -0.0484])  # the tote reading at seed shift 0

# -- motion ----------------------------------------------------------------------------------------------------------


def grip(pitch_deg: float, yaw_deg: float = 0.0) -> np.ndarray:
    """A grip whose approach axis points forward (+x) turned ``yaw`` about the vertical and pitched ``pitch`` down,
    the fingers closing in the vertical plane through the approach axis."""
    p, a = math.radians(pitch_deg), math.radians(yaw_deg)
    z = np.array([math.cos(p) * math.cos(a), math.cos(p) * math.sin(a), -math.sin(p)])
    side = np.array([-math.sin(a), math.cos(a), 0.0])  # horizontal, across the approach
    x = np.cross(side, z)  # finger closing axis: in the vertical plane through z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


PINCH = grip(60.0)  # reaches down to the strap with the wrist clear of the deck
CARRY_RING = grip(20.0)  # turned up while lifting: the arm reaches higher
PARK_Q = {"left": np.array([1.2, *READY[1:]]), "right": np.array([-1.2, *READY[1:]])}


class JointMove:
    """A smooth joint-space move to ``q`` with the gripper open (same interface as ``ArmPlan``)."""

    def __init__(self, duration: float, q, q_start):
        self.duration, self.q_goal, self.q_start = duration, np.asarray(q, float), np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.waypoints = [Waypoint(duration, (0.0, 0.0, 0.0), FINGER_OPEN)]
        self.worst_ik_mm = 0.0

    def action7(self, t: float):
        self.q = self.q_start + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q_start)
        return np.r_[self.q, 1.0], "park"


@dataclass
class Segment:
    """Waypoints per arm (an arm left out holds its last target), played together; then ``collect`` frames."""

    moves: dict  # arm -> list of Waypoint; a float: park the arm; ("ready", s): to the ready pose
    collect: object = None
    min_s: float = 0.0


# Plan geometry, all relative to the reference layout and moved by the measured shifts.
PINCH_BACK, PINCH_Z = 0.036, 0.022  # pinch the strap 3.6 cm short of the ring's centre, 2.2 cm above the deck
HALF_OPEN = -0.022  # fingers 44 mm apart, either side of the 25 mm strap; the lower one stays above the deck
LIFT = 0.115
OVER_RIM = 0.17  # carry the ring this high over the deck (the hanging ring clears the tote's 10 cm rim)
TOTE_FINAL_X = 0.23  # the tote's centre on the table (the loaded arms lag about 3 cm, so it lands near 0.26)
LEDGE_IN = 0.035  # TCP this far behind the ledge's front end
TOTE_LIFT = 0.03
SET_ABOVE = 0.004  # let go with the tote's bottom this high above the table


def strap_plan(ring_xy, u) -> list[Waypoint]:
    """Left hand: pinch the strap PINCH_BACK short of the ring (along the strap) and lift the ring off the tine."""
    g = np.array([*(np.asarray(ring_xy) - PINCH_BACK * np.asarray(u)), K.DECK_TOP + PINCH_Z])
    pre = g - 0.07 * PINCH[:, 2]
    return [Waypoint(2.0, tuple(pre), HALF_OPEN, "behind the strap", PINCH),
            Waypoint(1.2, tuple(g), HALF_OPEN, "onto the strap", PINCH),
            Waypoint(0.8, tuple(g), FINGER_CLOSED, "pinch", PINCH),
            Waypoint(1.5, tuple(g + (0, 0, LIFT)), FINGER_CLOSED, "lift the ring off the tine", CARRY_RING)]


def into_tote_plan(ring_xy, tote_xy) -> list[Waypoint]:
    """Raise the freed ring above the tote's rim, carry it over the tote and let it drop inside, so the strap travels
    with the tote."""
    high = K.DECK_TOP + OVER_RIM
    over = (tote_xy[0] + 0.03, tote_xy[1] + 0.07, high)
    return [Waypoint(1.0, (ring_xy[0], ring_xy[1], high), FINGER_CLOSED, "above the rim", CARRY_RING),
            Waypoint(1.8, over, FINGER_CLOSED, "over the tote", CARRY_RING),
            Waypoint(0.6, over, FINGER_OPEN, "drop the ring in", CARRY_RING),
            Waypoint(0.8, (over[0] - 0.06, over[1], high), FINGER_OPEN, "away", CARRY_RING)]


def ledge_targets(tote_xy, arm: str):
    """(TCP at the ledge (x, y, z) for the tote centred at tote_xy on the shelf, y offset of the ledge)."""
    tx, ty = tote_xy
    s = 1.0 if arm == "left" else -1.0
    y = ty + s * (K.TOTE_L / 2 + K.LEDGE_OUT / 2)
    return np.array([tx - K.LEDGE_LEN / 2 + LEDGE_IN, y, RIM - K.LEDGE_T / 2])


CARRY_PITCH = 30.0  # degrees down: a level grip cannot reach the table in front of the shelf (IK)


def carry_plan(tote_xy, arm: str, final_x=TOTE_FINAL_X, lift=TOTE_LIFT, hold=False) -> list[Waypoint]:
    rot = grip(CARRY_PITCH)
    g = ledge_targets(tote_xy, arm)
    dx = final_x - tote_xy[0]
    down = g + (dx, 0, (TOP + SET_ABOVE) - K.DECK_TOP)
    up = g + (0, 0, lift)
    steps = [Waypoint(2.0, tuple(g - (0.10, 0, 0)), FINGER_OPEN, "in front of the ledge", rot),
             Waypoint(1.2, tuple(g), FINGER_OPEN, "fingers round the ledge", rot),
             Waypoint(0.8, tuple(g), FINGER_CLOSED, "grip", rot),
             Waypoint(1.2, tuple(up), FINGER_CLOSED, "lift", rot),
             Waypoint(2.5, tuple(up + (dx, 0, 0)), FINGER_CLOSED, "draw it off the shelf", rot),
             Waypoint(2.0, tuple(down), FINGER_CLOSED, "lower onto the table", rot),
             Waypoint(0.5, tuple(down), FINGER_CLOSED, "settle", rot)]
    if hold:
        return steps + [Waypoint(4.0, tuple(down), FINGER_CLOSED, "keep holding", rot)]
    return steps + [Waypoint(0.8, tuple(down), FINGER_OPEN, "let go", rot),
                    Waypoint(1.2, tuple(down + (-0.04, 0, 0.08)), FINGER_OPEN, "back out and up", rot)]


class Controller:
    """Plays :meth:`script` segment by segment; returns (left 7-vector, right 7-vector, finished time or None)."""

    def __init__(self, bases, dt):
        self.dt = dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.q = {a: np.array(READY, float) for a in ARMS}
        self.finger = dict.fromkeys(ARMS, FINGER_OPEN)
        self.plans: dict = {}
        self.t0 = self.dur = 0.0
        self.collect, self.samples = None, []
        self.finished_at = None
        self.cam = None
        self.mode = os.environ.get("DEMO", "safe")
        self.worst_ik_mm = 0.0
        self.script_it = self.script()

    def say(self, msg: str) -> None:
        print(f"[totedemo] {msg}", flush=True)

    def _start(self, seg: Segment) -> None:
        for arm, plan in self.plans.items():
            self.worst_ik_mm = max(self.worst_ik_mm, plan.worst_ik_mm)
            self.q[arm], self.finger[arm] = plan.q.copy(), plan.waypoints[-1].finger
        self.plans = {a: JointMove(w, PARK_Q[a], self.q[a]) if isinstance(w, float) else
                      JointMove(w[1], READY, self.q[a]) if isinstance(w, tuple) else
                      ArmPlan(list(w), self.bases[a], q_start=self.q[a], finger_start=self.finger[a])
                      for a, w in seg.moves.items()}
        self.dur = max([p.duration for p in self.plans.values()] + [seg.min_s])
        self.collect, self.samples = seg.collect, []

    def act(self, obs):
        t = float(obs.state_time)
        if self.cam is None:
            h, w = obs.images["top_cam"].shape[:2]
            self.cam = TopCamera(w, h)
        if self.collect is not None:
            self.samples.append(self.collect(obs))
        while self.finished_at is None and t + self.dt >= self.t0 + self.dur:
            self.t0 += self.dur
            try:
                self._start(next(self.script_it))
            except StopIteration:
                self.t0 -= self.dur
                self.finished_at = t
                self.collect = None
                self.say(f"done at {t:.1f} s; worst IK error {self.worst_ik_mm:.1f} mm")
        out = []
        for arm in ARMS:
            if arm in self.plans:
                a7, _ = self.plans[arm].action7(min(t + self.dt - self.t0, self.plans[arm].duration))
            else:
                a7 = np.r_[self.q[arm], self.finger[arm] / FINGER_OPEN]
            out.append(a7)
        return out[0], out[1], self.finished_at

    def _look(self):
        yield Segment({}, collect=lambda obs: find_rig(self.cam, obs.images["top_cam"]), min_s=0.5)
        seen = [s for s in self.samples if s is not None]
        if not seen:
            raise RuntimeError("the tote and strap are not visible")
        tote = np.median([s[0] for s in seen], axis=0)
        ring = np.median([s[1] for s in seen], axis=0)
        u = np.median([s[2] for s in seen], axis=0)
        u /= np.linalg.norm(u)
        self.say(f"tote reading {np.round(tote, 4)}, ring at {np.round(ring, 4)}, strap direction {np.round(u, 2)}")
        return np.array(K.TOTE_XY) + tote - REF_TOTE, ring, u

    def script(self):
        yield Segment(dict.fromkeys(ARMS, 2.0))  # park both arms out of the camera's view
        tote_xy, ring_xy, u = yield from self._look()
        self.say(f"tote at {np.round(tote_xy, 3)}, ring at {np.round(ring_xy, 3)}")
        m = self.mode
        if m in ("safe", "hold"):
            yield Segment({"left": ("ready", 1.5)})
            yield Segment({"left": strap_plan(ring_xy, u) + into_tote_plan(ring_xy, tote_xy)})
            yield Segment({"left": ("ready", 1.5)})
            yield Segment({"left": 1.5})  # park again and look again: the strap's pull may have shifted the tote
            tote_xy, _, _ = yield from self._look()
            self.say(f"tote now at {np.round(tote_xy, 3)}")
        yield Segment(dict.fromkeys(ARMS, ("ready", 1.5)))
        final_x = {"short": K.DECK_X[0] - K.TOTE_W / 2 - 0.01}.get(m, TOTE_FINAL_X)
        lift = 0.06 if m == "yank" else TOTE_LIFT
        yield Segment({a: carry_plan(tote_xy, a, final_x, lift, hold=m == "hold") for a in ARMS})
