"""On the station table, a little left of centre: a loose light-wood block (11 x 7 x 5.5 cm, long side along y,
left-right) with a square blind bore (2.4 x 2.4 cm, 9 cm deep) running horizontally into its right end face. A wooden
peg (a blue round shaft 2 cm across and 8.5 cm long, with a square 4 x 3.4 x 3.4 cm head) rests in the bore with 4 cm of
its shaft inside, its head toward the right arm. The peg is fully in when its head touches the block's end face. To
the left of the block is an empty grey parts tray (14 x 14 cm, 1.5 cm rims).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from pegkit import PartState, ball, contact_force, on_physics, on_reset, part, rot_x, rot_z  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Push the peg fully into the block."

WOOD, PEG_WOOD, STEEL, TRAY_GREY = (0.80, 0.62, 0.40), (0.55, 0.33, 0.16), (0.72, 0.74, 0.78), (0.30, 0.32, 0.36)
SHAFT_BLUE = (0.15, 0.40, 0.80)       # the shaft (the part that goes in) is painted blue

# -- the block (part frame: origin on the table under its centre; the bore runs along x and opens toward -x). The
# block, peg and ball are turned YAW = 90 degrees about z, so in the world the bore runs along y and opens toward -y
# (the right arm). Scoring works in the block's frame; the demo works in world coordinates. ----------------------
BL, BW, BH = 0.11, 0.07, 0.055       # 11 cm along the part x (world y), 7 cm wide, 5.5 cm tall
S = 0.024                             # square bore, 2.4 cm
ZC = 0.0275                           # bore axis 2.75 cm above the table
MOUTH, END = -BL / 2, 0.035           # bore from the -x face (x -5.5 cm) to its closed end (x +3.5 cm): 9 cm deep
_LOW, _HIGH = ZC - S / 2, ZC + S / 2
_SIDE = (BW - S) / 2
BLOCK_BOXES = [
    ((0, 0, _LOW / 2), (BL, BW, _LOW), WOOD),                                   # under the bore
    ((0, 0, (_HIGH + BH) / 2), (BL, BW, BH - _HIGH), WOOD),                     # over the bore
    (((MOUTH + END) / 2, S / 2 + _SIDE / 2, ZC), (END - MOUTH, _SIDE, S), WOOD),  # bore side walls
    (((MOUTH + END) / 2, -S / 2 - _SIDE / 2, ZC), (END - MOUTH, _SIDE, S), WOOD),
    (((END + BL / 2) / 2, 0, ZC), (BL / 2 - END, S, S), WOOD),                  # the bore's closed end, 2 cm
]
BX, BY = 0.31, 0.04
YAW = math.pi / 2                     # part frame (x, y) -> world (-y, x)
BLOCK = part("block", pos=(BX, BY, TOP), boxes=BLOCK_BOXES, mass=0.20, yaw=YAW, color=WOOD, friction=0.3)

# -- the peg (part frame: origin at the centre of the shaft's tip; the shaft runs toward -x, world -y) ------------
P, SHAFT = 0.020, 0.085               # 2 cm diameter round shaft, 8.5 cm long
HEAD, HEAD_L = 0.034, 0.040           # square head 3.4 x 3.4 cm, 4 cm long (flat faces for a parallel grip)
PEG_BOXES = [((-SHAFT / 2, 0, 0), (SHAFT, P, P), SHAFT_BLUE, "cyl"),                 # round shaft
             ((-SHAFT - HEAD_L / 2, 0, 0), (HEAD_L, HEAD, HEAD), PEG_WOOD)]
START_IN = 0.040                      # 4 cm of shaft inside the bore at the start
PEG = part("peg", pos=(BX, BY + MOUTH + START_IN, TOP + ZC), boxes=PEG_BOXES, mass=0.05, yaw=YAW,
           color=PEG_WOOD, friction=0.3)

# -- the steel ball, hidden at the bottom of the bore ---------------------------------------------------------------
R = 0.010                             # 20 mm steel ball, 33 g
BALL = ball("ball", pos=(BX, BY + END - R - 0.0005, TOP + _LOW + R + 0.0005), radius=R, mass=0.033, color=STEEL,
            friction=0.4, report=True)
# -- a shallow open parts tray to the left of the block ---------------------------------------------------------------
TX, TY = BX - 0.06, BY + 0.14
T_HALF, T_WALL, T_H = 0.07, 0.004, 0.015  # 14 x 14 cm, 1.5 cm rims
TRAY = part("tray", pos=(TX, TY, TOP), boxes=[((0, 0, 0.0015), (2 * T_HALF, 2 * T_HALF, 0.003), TRAY_GREY)]
            + [((sx * (T_HALF - T_WALL / 2), 0, T_H / 2), (T_WALL, 2 * T_HALF, T_H), TRAY_GREY) for sx in (1, -1)]
            + [((0, sy * (T_HALF - T_WALL / 2), T_H / 2), (2 * T_HALF - 2 * T_WALL, T_WALL, T_H), TRAY_GREY)
               for sy in (1, -1)], mass=0.15, color=TRAY_GREY, friction=0.5)
OBJECTS = [BLOCK, PEG, BALL, TRAY]

FULL_IN = SHAFT                       # head on the end face: the tip is 8.5 cm in, 5 mm short of the bore's end
BLOCKED_IN = END - 2 * R - MOUTH      # with the ball in: the tip stops 7 cm in, head 1.5 cm proud

# -- damage: forcing the peg onto the ball --------------------------------------------------------------------------
FORCE_LIMIT = 3.0                     # N, peg on ball
FORCE_STEPS = 3                       # physics steps in a row (12.5 ms at 240 Hz)

_STATE: dict = {}


def _reset_state() -> None:
    _STATE.update(n=0, over=0, damaged=False, peak=0.0, lost=False)


_reset_state()
on_reset(_reset_state)


@on_physics
def _force_check(environment) -> None:
    """Before every physics step: the peg-ball contact force of the last step; damage once it has been over
    FORCE_LIMIT for FORCE_STEPS steps in a row."""
    f = contact_force("ball", "peg")
    _STATE["peak"] = max(_STATE["peak"], f)
    _STATE["over"] = _STATE["over"] + 1 if f > FORCE_LIMIT else 0
    if _STATE["over"] >= FORCE_STEPS:
        _STATE["damaged"] = True
    _STATE["n"] += 1
    if os.environ.get("PEG_DEBUG") and _STATE["n"] % 24 == 0:  # PEG_DEBUG=1: print the state every 0.1 s
        from yam_arena.scene import object_states

        o = object_states(environment, ["block", "peg", "ball"])
        blk = o["block"]
        print(f"[peg] t {_STATE['n'] / 240:.1f} block {np.round(blk.pos, 3)} tilt {math.degrees(blk.tilt()):.0f} "
              f"depth {depth(blk, o['peg']) * 100:.1f} cm peg {np.round(o['peg'].pos, 3)} "
              f"pitch {math.degrees(math.asin(o['peg'].rotation()[2, 0])):.1f} "
              f"ball {np.round(blk.to_local(o['ball'].pos), 3)} "
              f"force {f:.1f} peak {_STATE['peak']:.1f} damaged {_STATE['damaged']}", flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------
FULL_TOL = 0.003                      # head within 3 mm of the end face
PART_IN = 0.040                       # 4 cm of shaft in the bore
OFF_TABLE = 0.05                      # 5 cm below the table top: fallen off


def depth(block: PartState, peg: PartState) -> float:
    """How far (m) the peg's tip is inside the bore, along the bore's axis from the mouth; negative if outside or not
    lined up with the bore (tip more than 1 cm off its axis)."""
    tip = block.to_local(peg.pos)
    if abs(tip[1]) > 0.01 or abs(tip[2] - ZC) > 0.01:
        return -1.0
    return float(tip[0] - MOUTH)


def ball_out(block: PartState, b) -> bool:
    """The ball's centre is outside the block's outer box."""
    x, y, z = block.to_local(b.pos)
    return not (abs(x) < BL / 2 and abs(y) < BW / 2 and 0 < z < BH)


def score(objects) -> float:
    """0 if the peg was ever forced onto the ball (over FORCE_LIMIT for 12.5 ms) or the block, peg or ball ever fell off
    the table. Otherwise 1.0 with the ball out of the block and the peg's head
    within 3 mm of the end face; 0.5 with the ball out and 4 cm or more of the shaft in the bore; 0.25 with the ball
    out; else 0."""
    block, peg, b = objects["block"], objects["peg"], objects["ball"]
    _STATE["lost"] |= any(objects[n].pos[2] < TOP - OFF_TABLE for n in ("block", "peg", "ball"))
    if _STATE["damaged"] or _STATE["lost"] or not ball_out(block, b):
        return 0.0
    d = depth(block, peg)
    if d >= FULL_IN - FULL_TOL:
        return 1.0
    return 0.5 if d >= PART_IN else 0.25


def success(objects) -> bool:
    """Ball out, peg fully in, nothing damaged or lost; held 1 s."""
    return score(objects) == 1.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
OPEN, SHUT = -0.04695, 0.0            # finger joint targets (m): yam_arena.constants FINGER_OPEN / FINGER_CLOSED
# Both grips close the fingers along x. Straight down, the two wrists (link4) collide over the block, so each hand
# leans LEAN outward about x (the finger-closing axis, so the pads still hold the same faces): right wrist toward -y,
# left wrist toward +y. The pad tips then sit TCP_PAST * cos(LEAN) below and TCP_PAST * sin(LEAN) inward of the TCP.
LEAN = math.radians(25)
TCP_PAST = 0.013 * math.cos(LEAN)     # the TCP is 13 mm short of the pad tips
LEAN_DY = 0.013 * math.sin(LEAN)


def down(yaw: float) -> np.ndarray:
    """yam_arena.demo.grip_rotation(yaw) as a matrix (straight down, yaw 0 closes the fingers along world y, yaw 90
    degrees along world x)."""
    z = np.array([0.0, 0.0, -1.0])
    x = np.array([0.0, -1.0, 0.0])
    return rot_z(yaw) @ np.column_stack([x, np.cross(z, x), z])


GRIP_BLOCK = (BX, BY + 0.02 + LEAN_DY, TOP + BH - 0.015 + TCP_PAST)   # pad tips 1.5 cm below the block's top
TILT = math.radians(30)               # about the world x axis: the mouth (-y) swings down 30 degrees
# The move to the tray runs along the bore, and its acceleration would roll the ball out of the mouth early (and off
# the table); the block is carried mouth-up by CARRY so the ball stays against the bore's end until the tip.
CARRY = math.radians(-15)
# Tipped 10 to 30 degrees, the mouth is about 6 cm to the right (-y) and 4 to 6 cm below the grip. The ball leaves it
# rolling toward -y, so the mouth is held 3 cm on the +y side of the tray's centre, 5 cm above the tray.
TIP_AT = (TX, TY + 0.09, TOP + 0.115)
SHOW_AT = (TX + 0.13, TY + 0.09, TOP + 0.17)  # block held up and forward, off the top camera's view of the tray


def head_y(depth_in: float) -> float:
    """World y of the right grip on the peg's head when the shaft is ``depth_in`` into the bore: 1 cm beyond the
    head's centre, so the fingers stay clear of the block when the head meets it."""
    return BY + MOUTH + depth_in - SHAFT - HEAD_L / 2 - 0.01 - LEAN_DY


# Calibrated in simulation: the block does not come back exactly to its start after the tip and set-down, and the
# peg hangs about 3 degrees nose-up in the grip; this open-loop plan cannot look, so the second insertion aims
# SETTLE_DX off the bore's start line and 4 mm lower.
SETTLE_DX = -0.006
FORCE_PUSH = 0.10                     # DEMO_FORCE: 10 cm past the stop
PUSH_PAST = 0.0075                    # the demo: 7.5 mm past the stop (the arm lags about 4 mm; peg-ball force under 1.5 N)
# The resting peg droops in the bore's clearance (tip up against the ceiling, head about 9 mm low); pushed like that
# it wedges. The demo grips the drooping head and lifts it level before pushing.
HEAD_Z = TOP + ZC - 0.009 - 0.008 + TCP_PAST  # pad tips 8 mm below the drooping head's centre
LEVEL_Z = HEAD_Z + 0.009
REINSERT_Z = LEVEL_Z - 0.004


def plan(force: bool = False) -> dict[str, list]:
    """(duration, pos, rot, finger, label) per arm, all grips straight down with the fingers closing along x.
    Right: grip the peg's head, push until the peg stops, pull it out and wait aside. Left: hold the block, lift
    it, tip it mouth-down so the ball rolls out, put it back and hold it. Right: push the peg fully in.
    ``force``: push FORCE_PUSH past the stop instead of PUSH_PAST."""
    g = rot_x(LEAN) @ down(math.pi / 2)        # right arm; the left arm turns the other way (wrist range)
    gl = rot_x(-LEAN) @ down(-math.pi / 2)
    push = BLOCKED_IN + (FORCE_PUSH if force else PUSH_PAST)
    out_y = head_y(-0.015)
    aside = (BX - 0.06, out_y - 0.03, TOP + 0.12)
    right = [(2.5, (BX, head_y(START_IN), TOP + 0.12), g, OPEN, "over the peg head"),
             (1.5, (BX, head_y(START_IN), HEAD_Z), g, OPEN, "down to the head"),
             (0.8, (BX, head_y(START_IN), HEAD_Z), g, SHUT, "grip the head"),
             (1.0, (BX, head_y(START_IN), LEVEL_Z), g, SHUT, "lift the head: the peg levels in the bore"),
             (2.5, (BX, head_y(push), LEVEL_Z), g, SHUT, "push the peg in"),
             (1.5, (BX, head_y(push), LEVEL_Z), g, SHUT, "it stops short: hold"),
             (2.5, (BX, out_y, LEVEL_Z), g, SHUT, "pull it out"),
             (1.0, (BX, out_y, TOP + 0.12), g, SHUT, "lift it clear"),
             (1.5, aside, g, SHUT, "aside")]
    bx, by, bz = GRIP_BLOCK
    rest = (bx, by, bz + 0.002)
    left = [(2.5, (bx, by, TOP + 0.12), gl, OPEN, "over the block"),
            (1.5, GRIP_BLOCK, gl, OPEN, "down to the block"),
            (0.8, GRIP_BLOCK, gl, SHUT, "grip the block")]
    left.append((sum(w[0] for w in right[:7]) - sum(w[0] for w in left), GRIP_BLOCK, gl, SHUT, "hold the block"))
    tipped, carried = rot_x(TILT) @ gl, rot_x(CARRY) @ gl
    tip = [(1.5, (bx, by, TOP + 0.115), carried, SHUT, "lift it, mouth up"),
           (1.5, TIP_AT, carried, SHUT, "move it aside"),
           (1.5, TIP_AT, tipped, SHUT, "tip it mouth-down over the tray"),
           (2.5, TIP_AT, tipped, SHUT, "let the ball roll out"),
           (1.5, TIP_AT, gl, SHUT, "level"),
           (1.0, SHOW_AT, gl, SHUT, "lift it clear of the tray"),
           (1.0, SHOW_AT, gl, SHUT, "pause: the ball lies in the tray"),
           (1.5, (bx, by, TOP + 0.10), gl, SHUT, "back over its place"),
           (1.5, rest, gl, SHUT, "set it down"),
           (0.5, rest, gl, SHUT, "settle")]
    left += tip
    IX = BX + SETTLE_DX
    insert = [(1.5, (IX, out_y, TOP + 0.12), g, SHUT, "back in front of the bore"),
              (1.5, (IX, out_y, REINSERT_Z), g, SHUT, "down to the bore's height"),
              (1.5, (IX, head_y(0.03), REINSERT_Z), g, SHUT, "into the bore"),
              (2.5, (IX, head_y(FULL_IN + 0.015), REINSERT_Z), g, SHUT, "push it fully in"),
              (1.0, (IX, head_y(FULL_IN + 0.015), REINSERT_Z), g, SHUT, "hold"),
              (0.8, (IX, head_y(FULL_IN + 0.015), REINSERT_Z), g, OPEN, "let go"),
              (1.5, (IX, head_y(FULL_IN), TOP + 0.14), g, OPEN, "away")]
    right.append((sum(w[0] for w in left) - sum(w[0] for w in right), aside, g, SHUT, "wait aside"))
    right += insert
    left += [(sum(w[0] for w in insert[:5]), rest, gl, SHUT, "hold it while the peg goes in"),
             (0.8, rest, gl, OPEN, "let go"),
             (1.5, (bx, by, TOP + 0.14), gl, OPEN, "away")]
    return {"left": left, "right": right}


def demo():
    """Right arm pushes the peg until it stops short, pulls it out; left arm tips the block over the tray so the
    hidden ball rolls out into it and puts the block back; right arm pushes the peg fully in. DEMO_FORCE=1 pushes
    FORCE_PUSH past the stop instead: the peg-ball force passes FORCE_LIMIT and the trial scores 0."""
    from yam_arena.demo import Waypoint

    p = plan(force=bool(os.environ.get("DEMO_FORCE")))
    return {arm: [Waypoint(d, tuple(pos), f, label, r) for d, pos, r, f, label in wps] for arm, wps in p.items()}
