"""Scripted demo for sterile-peel-pouch, played by ``yam_scripted`` (see ``pouchkit._patch_scripted``).

Open loop on the fixed layout (the task has no seeded shift): it plans from the layout constants and never reads the
simulator's state. The pouch lies flat on its rest, paper up. The left hand pinches the upturned paper tab's front
corner, lifts the chevron end and turns the tab level, so the pouch slides off the rest and hangs from that tab with
the film tab turned out toward the right. The right hand, turned 40 degrees in toward the middle, fingers above and
below the film tab, pinches its front corner. Both carry the pouch over the field, then draw apart slowly, coming
down a little, so the seal parts from the chevron down; the forceps slides out of the opened mouth onto the drape.
The hands then carry the two empty plies out beyond the drape's side edges, lay them on the table, let go and go back
to the ready pose.

Contrasts (``DEMO=...``): ``fast`` peels in 1.2 s (a jerk); ``low`` carries the pouch too low, so it drags on the
drape; ``grab`` pinches the flat pouch's sealed body (both plies together) at its front edge in one hand, shakes it over
the field and drops it there; ``short`` stops the peel half way and releases the pouch over the field.
"""

from __future__ import annotations

import math
import os

import numpy as np
import pouchkit as K
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import ArmPlan, Waypoint

ARMS = ("left", "right")
TOP = K.TOP
YAW = 40.0  # degrees: each hand's approach turned in toward the middle
HALF_OPEN = -0.020  # fingers 40 mm apart round the 1.5 mm tab
GRIP_BACK = 0.027  # TCP this far behind the tab's front edge (x)
GRIP_Y = 0.020  # TCP this far out from the pouch's mid-plane (the pad tips end 8 mm out, clear of the ply)
GRIP_Z = K.Z_SEAL_TOP + K.PLY_T / 2  # GRIP_* are in the upright design frame (pouchkit.flat)
PICK_DZ = 0.003  # the left hand pinches the upturned paper tab 3 mm higher, so its fingers clear the paper ply
CARRY_Z = TOP + 0.30  # the hanging pouch's bottom 9 cm up: clear of the rest and of the drape
RIGHT_WAIT = 6.5  # s: the right arm waits at ready while the left lifts the pouch and it settles
FIELD_X = 0.45  # TCP x over the field; the pouch hangs at x 0.47 and the forceps slides about 9 cm back as it drops
PEEL_Y = 0.26  # each hand this far out at the end of the peel
PEEL_END_Z = TOP + 0.27  # and this high: once the plies part, each hangs 23 cm below its hand
PEEL_S = 6.0
DISCARD = (0.35, 0.45, TOP + 0.10)  # lay the plies down here, 12 cm beyond the drape's side edges (y +-0.33), and let go


def grip(side: float, yaw_deg: float = YAW) -> np.ndarray:
    """A level grip whose approach points forward, turned ``yaw`` toward the middle; fingers close vertically.
    side +1: left arm (turns toward -y), -1: right arm."""
    a = math.radians(yaw_deg)
    z = np.array([math.cos(a), -side * math.sin(a), 0.0])
    x = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


class JointMove:
    """A smooth joint-space move to ``q`` (same interface as ``ArmPlan``)."""

    def __init__(self, duration: float, q, q_start, finger):
        self.duration, self.q_goal, self.q_start = duration, np.asarray(q, float), np.asarray(q_start, float)
        self.q = self.q_start.copy()
        self.waypoints = [Waypoint(duration, (0.0, 0.0, 0.0), finger)]
        self.worst_ik_mm = 0.0

    def action7(self, t: float):
        self.q = self.q_start + ik.smoothstep(min(t / self.duration, 1.0)) * (self.q_goal - self.q_start)
        return np.r_[self.q, self.waypoints[0].finger / FINGER_OPEN], "joint move"


def arm_plan(side: float, mode: str, ready) -> list[Waypoint]:
    """``ready``: (TCP position, rotation) of the arm's ready pose."""
    s = side
    R = grip(s)
    a = R[:, 2]
    carry_z = TOP + 0.20 if mode == "low" else CARRY_Z
    gx = K.POUCH_X - K.POUCH_W / 2 + GRIP_BACK
    up = np.array([gx, s * GRIP_Y, carry_z])  # the pinch once the pouch hangs from the left hand
    if s > 0:  # left: take the upturned paper tab of the flat pouch, lift it and turn the tab level
        Rp = K.FLAT_R @ R
        gp = K.flat((gx, GRIP_Y, GRIP_Z)) + (0, 0, PICK_DZ)
        w = [Waypoint(2.5, tuple(gp - 0.07 * Rp[:, 2]), HALF_OPEN, "in front of the paper tab", Rp),
             Waypoint(1.5, tuple(gp), HALF_OPEN, "fingers round the tab", Rp),
             Waypoint(0.8, tuple(gp), FINGER_CLOSED, "pinch the tab", Rp),
             Waypoint(1.0, tuple(gp + (0, 0, 0.03)), FINGER_CLOSED, "lift the chevron end", Rp),
             Waypoint(2.5, tuple(up), FINGER_CLOSED, "lift the pouch off the rest, tab level", R),
             Waypoint(RIGHT_WAIT + 4.8 - 8.3, tuple(up), FINGER_CLOSED, "hold while the right hand takes the film tab",
                      R)]
    else:  # right: once the pouch hangs, its film tab sticks out level toward -y; pinch its front corner
        w = [Waypoint(RIGHT_WAIT, tuple(ready[0]), FINGER_OPEN, "wait", ready[1]),
             Waypoint(2.5, tuple(up - 0.07 * a), HALF_OPEN, "in front of the film tab", R),
             Waypoint(1.5, tuple(up), HALF_OPEN, "fingers round the tab", R),
             Waypoint(0.8, tuple(up), FINGER_CLOSED, "pinch the tab", R)]
    over = np.array([FIELD_X, up[1], carry_z])
    peel_y = s * (PEEL_Y if mode != "short" else 0.10)
    apart = np.array([FIELD_X, peel_y, carry_z if mode == "short" else PEEL_END_Z + carry_z - CARRY_Z])
    w += [Waypoint(2.5, tuple(over), FINGER_CLOSED, "over the field", R),
          Waypoint(1.2 if mode == "fast" else PEEL_S, tuple(apart), FINGER_CLOSED, "peel apart", R),
          Waypoint(1.5, tuple(apart), FINGER_CLOSED, "hold while the forceps drops", R)]
    if mode == "short":
        return w + [Waypoint(0.6, tuple(apart), FINGER_OPEN, "let go over the field", R),
                    Waypoint(1.5, tuple(apart - 0.06 * a + (0, 0, 0.04)), FINGER_OPEN, "back", R)]
    back = np.array([DISCARD[0], s * DISCARD[1], apart[2]])
    down = np.array([DISCARD[0], s * DISCARD[1], DISCARD[2]])
    return w + [Waypoint(3.0, tuple(back), FINGER_CLOSED, "carry the plies off the field", R),
                Waypoint(1.0, tuple(down), FINGER_CLOSED, "lay them on the table", R),
                Waypoint(0.6, tuple(down), FINGER_OPEN, "let go", R),
                Waypoint(1.0, tuple(down + (0, 0, 0.06)), FINGER_OPEN, "up", R)]


def grab_plan() -> list[Waypoint]:
    """Cheat: the left hand pinches the flat pouch's sealed body (both plies) at its front edge, where it overhangs the
    rest, lifts it, shakes it over the field and lets go; the right arm stays at ready."""
    a = math.radians(30.0)  # upright frame: approach forward turned 30 degrees toward -y, fingers across the plies
    R = K.FLAT_R @ np.column_stack([[math.sin(a), math.cos(a), 0], [0, 0, 1.0], [math.cos(a), -math.sin(a), 0]])
    R = R @ np.diag([-1.0, -1.0, 1.0])  # the same grip turned over about the approach: within the wrist's reach
    g = K.flat((K.POUCH_X - K.POUCH_W / 2 + 0.006, 0.0, K.Z_SEAL_TOP - 0.05))
    up = np.array([g[0], g[1], CARRY_Z])
    over = np.array([FIELD_X, 0.0, CARRY_Z])
    w = [Waypoint(2.5, tuple(g - 0.08 * R[:, 2]), HALF_OPEN, "in front of the pouch", R),
         Waypoint(1.5, tuple(g), HALF_OPEN, "fingers round the pouch", R),
         Waypoint(0.8, tuple(g), FINGER_CLOSED, "grab it", R),
         Waypoint(2.0, tuple(up), FINGER_CLOSED, "lift", R),
         Waypoint(2.5, tuple(over), FINGER_CLOSED, "over the field", R)]
    for i in range(6):
        w.append(Waypoint(0.35, tuple(over + (0, 0.04 * (-1) ** i, -0.03 * (i % 2))), FINGER_CLOSED, "shake", R))
    return w + [Waypoint(0.6, tuple(over), FINGER_OPEN, "let go", R), Waypoint(1.0, tuple(over - (0.06, 0, 0)), FINGER_OPEN, "back", R)]


class Controller:
    """Plays the plan; returns (left 7-vector, right 7-vector, finished time or None)."""

    def __init__(self, bases, dt):
        self.dt = dt
        self.bases = {a: np.asarray(bases[a], float) for a in ARMS}
        self.mode = os.environ.get("DEMO", "safe")
        if self.mode == "grab":
            self.plans = {"left": ArmPlan(grab_plan(), self.bases["left"], q_start=READY)}
        else:
            self.plans = {a: ArmPlan(arm_plan(s, self.mode, ik.tcp_pose(np.asarray(READY, float), self.bases[a])),
                                     self.bases[a], q_start=READY)
                          for a, s in (("left", 1.0), ("right", -1.0))}
        self.dur = max(p.duration for p in self.plans.values())
        self.home = None
        self.finished_at = None
        self.say(f"mode {self.mode}, plan {self.dur:.1f} s")

    def say(self, msg: str) -> None:
        print(f"[peeldemo] {msg}", flush=True)

    def act(self, obs):
        t = float(obs.state_time) + self.dt
        out = []
        for arm in ARMS:
            plan = self.plans.get(arm)
            if plan is None:
                out.append(np.r_[READY, 1.0])
                continue
            if t <= self.dur:
                a7, _ = plan.action7(min(t, plan.duration))
            else:
                if self.home is None:
                    self.home = {a: JointMove(2.0, READY, p.q, FINGER_OPEN) for a, p in self.plans.items()}
                a7, _ = self.home[arm].action7(t - self.dur)
            out.append(a7)
        if self.finished_at is None and t > self.dur + 2.0:
            self.finished_at = float(obs.state_time)
            worst = max(p.worst_ik_mm for p in self.plans.values())
            self.say(f"done at {self.finished_at:.1f} s; worst IK error {worst:.1f} mm")
        return out[0], out[1], self.finished_at
