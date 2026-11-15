"""Make up a batch of cleaning solution in a clear 7.6 L tank: four 500 mL bottles of concentrate go in, and the
tank's water inlet, already running, brings the batch up to the fill line. A float switch at the fill line lights a
red lamp behind the tank. The inlet is a 1/2-inch ball valve with a quarter-turn lever on the pipe to the right of
the tank; it stays open until it is turned. The level rises with every bottle poured and with the inlet, so when the
lamp comes on depends on how fast the robot pours and on the start level (set by the seed).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""
# Idea first proposed in robocurve/benchmark-tasks#380; independent implementation.

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import tankkit  # noqa: E402
from tankkit import Hinge, on_reset, on_step, part, set_visible  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Make up the batch: empty the four bottles of concentrate into the tank and fill it to the line. "
               "Don't let it overflow or spill.")

WHITE, DARK, STEEL, BRASS = (0.93, 0.93, 0.91), (0.18, 0.19, 0.21), (0.62, 0.64, 0.67), (0.78, 0.62, 0.25)
YELLOW, BLUE, RED, LAMP_OFF = (0.95, 0.80, 0.10), (0.20, 0.45, 0.90), (1.0, 0.06, 0.04), (0.30, 0.10, 0.10)
CLEAR = (1.0, 1.0, 1.0, 0.0)  # collides, not drawn (the tank's clear polycarbonate walls)

# -- the tank (origin: centre of its footprint on the table) --------------------------------------------------------
TX, TY = 0.40, 0.0
W, WALL, FLOOR, H = 0.20, 0.006, 0.01, 0.20     # 20 x 20 cm inside, rim 20 cm up, inside depth 19 cm (7.6 L)
AREA = W * W                                    # m^2: 1 cm of depth is 400 mL
LINE = 0.13                                     # fill line / float switch: 13 cm of liquid (5.2 L)
RIM = H - FLOOR                                 # 19 cm of liquid: over the rim (overflow)
LAYER = 0.005                                   # the liquid is drawn in 5 mm layers
N_LAYERS = round(RIM / LAYER)
OUT = W / 2 + WALL
PIPE_Z, PIPE_T = 0.235, 0.022                   # inlet pipe: centre height, cross-section
PIVOT = (-0.04, -0.19, PIPE_Z + PIPE_T / 2 + 0.012)   # the valve's stem axis (top of the valve body)
SPOUT_Y = -0.075
LAMP_AT = (OUT + 0.02, -0.07)
LAMP_Z = 0.29

TANK_BOXES = [
    ((0, 0, FLOOR / 2), (2 * OUT, 2 * OUT, FLOOR), WHITE),                       # 0 floor
    ((W / 2 + WALL / 2, 0, H / 2), (WALL, 2 * OUT, H), WHITE),                     # 1 back wall (drawn)
    ((-W / 2 - WALL / 2, 0, H / 2), (WALL, 2 * OUT, H), CLEAR),                    # 2 front wall
    ((0, W / 2 + WALL / 2, H / 2), (W, WALL, H), CLEAR),                           # 3 left wall
    ((0, -W / 2 - WALL / 2, H / 2), (W, WALL, H), CLEAR),                          # 4 right wall
    ((-W / 2 - WALL / 2, 0, H - 0.004), (WALL, 2 * OUT, 0.008), DARK),             # 5-8 rim edges (drawn)
    ((0, W / 2 + WALL / 2, H - 0.004), (W, WALL, 0.008), DARK),
    ((0, -W / 2 - WALL / 2, H - 0.004), (W, WALL, 0.008), DARK),
    ((W / 2 + WALL / 2, 0, H - 0.004), (WALL, 2 * OUT, 0.008), DARK),
    ((W / 2 - 0.0005, 0, FLOOR + LINE), (0.001, W, 0.004), DARK),                 # 9 fill line on the back wall
    ((-W / 2 - WALL / 2, -W / 2 - WALL / 2, H / 2), (WALL, WALL, H), DARK),       # 10-11 front corner posts
    ((-W / 2 - WALL / 2, W / 2 + WALL / 2, H / 2), (WALL, WALL, H), DARK),
    # 12-14 the inlet: pipe from the right edge of the table, over the wall, and a spout turned down into the tank
    ((PIVOT[0], (-0.43 + SPOUT_Y) / 2, PIPE_Z), (PIPE_T, -SPOUT_Y + 0.43, PIPE_T), STEEL),
    ((PIVOT[0], SPOUT_Y, PIPE_Z - 0.025), (PIPE_T, PIPE_T, 0.03), STEEL),
    ((PIVOT[0], -0.43, PIPE_Z / 2), (0.03, 0.03, PIPE_Z), STEEL),               # pipe support at the table edge
    ((PIVOT[0], PIVOT[1], PIPE_Z), (0.036, 0.05, 0.036), BRASS),                 # 15 valve body
    ((LAMP_AT[0], LAMP_AT[1], LAMP_Z / 2), (0.012, 0.012, LAMP_Z), DARK),         # 16 lamp post
    ((LAMP_AT[0], LAMP_AT[1], LAMP_Z + 0.018), (0.04, 0.04, 0.036), LAMP_OFF),    # 17 lamp, unlit
    ((LAMP_AT[0], LAMP_AT[1], LAMP_Z + 0.018), (0.043, 0.043, 0.039), RED),       # 18 lamp, lit (shown at alarm)
    ((PIVOT[0], SPOUT_Y, (PIPE_Z - 0.04 + FLOOR) / 2), (0.01, 0.01, PIPE_Z - 0.04 - FLOOR), BLUE),  # 19 stream
]
LAMP_LIT, STREAM = 18, 19
FIRST_LAYER = len(TANK_BOXES)
TANK_BOXES += [((0, 0, FLOOR + LAYER * (i + 0.5)), (W - 0.002, W - 0.002, LAYER), BLUE) for i in range(N_LAYERS)]
GHOSTS = (9, LAMP_LIT, STREAM) + tuple(range(FIRST_LAYER, FIRST_LAYER + N_LAYERS))

# The lever, in its own frame (origin on the stem axis, lever open: along the pipe, pointing away from the tank).
# It turns about the vertical, 0 (open) to 90 degrees (closed: across the pipe, pointing away from the arms).
LEVER_LEN, SLEEVE = 0.115, (0.018, 0.06, 0.018)
LEVER_BOXES = (
    ((0, 0, 0.004), (0.014, 0.014, 0.008), STEEL),                                # nut on the stem
    ((0, -LEVER_LEN / 2, 0.004), (0.016, LEVER_LEN, 0.004), STEEL),               # flat steel lever
    ((0, -LEVER_LEN + SLEEVE[1] / 2, 0.006), SLEEVE, YELLOW),                     # vinyl grip
)
CLOSED_AT = math.radians(85)                    # sealed from 85 degrees (the stop is at 90)
LEVER = Hinge("lever", anchor=PIVOT, boxes=LEVER_BOXES, lower=0.0, upper=math.pi / 2, axis="Z",
              stiffness=0.0, damping=1.0, mass=0.06)
TANK = part("tank", pos=(TX, TY, TOP), boxes=TANK_BOXES, mass=6.0, hinge=LEVER, color=WHITE, friction=0.9,
            ghosts=GHOSTS, glows=(LAMP_LIT,))

# -- the bottles: 500 mL wide-mouth HDPE, 7 cm across, 18 cm tall, caps off ----------------------------------------
B_R, B_H, B_MASS = 0.035, 0.18, 0.54
MOUTH_R = 0.025                                 # the 53 mm mouth's edge, 2.5 cm off the axis
FILL = 0.0005                                   # m^3 (500 mL) in each
BOTTLE_XY = [(0.40, 0.18), (0.40, 0.30), (0.49, 0.18), (0.49, 0.30)]
BOTTLE_NAMES = [f"bottle_{i + 1}" for i in range(4)]
BOTTLES = [Cylinder(n, pos=(x, y, TOP + B_H / 2), radius=B_R, height=B_H, mass=B_MASS, color=WHITE)
           for n, (x, y) in zip(BOTTLE_NAMES, BOTTLE_XY)]
# A grey plastic bin for the empties: 20 x 24 cm inside, 8 cm deep.
BIN_XY, BIN_IN, BIN_H, BIN_T = (0.30, 0.472), (0.20, 0.24), 0.08, 0.006
GREY = (0.45, 0.47, 0.50)
BIN = part("bin", pos=(*BIN_XY, TOP), mass=1.5, color=GREY, friction=0.8, boxes=[
    ((0, 0, BIN_T / 2), (BIN_IN[0] + 2 * BIN_T, BIN_IN[1] + 2 * BIN_T, BIN_T), GREY),
    *[((s * (BIN_IN[0] + BIN_T) / 2, 0, BIN_H / 2), (BIN_T, BIN_IN[1] + 2 * BIN_T, BIN_H), GREY) for s in (-1, 1)],
    *[((0, s * (BIN_IN[1] + BIN_T) / 2, BIN_H / 2), (BIN_IN[0], BIN_T, BIN_H), GREY) for s in (-1, 1)]])
OBJECTS = [TANK, *BOTTLES, BIN]

# -- liquid -----------------------------------------------------------------------------------------------------------
INFLOW = 40e-6                                  # m^3/s with the valve fully open: 2.4 L/min (1 mm/s in the tank)
POUR_MAX = 100e-6                               # m^3/s through a bottle's mouth (glugging)
START_LEVELS = (0.020, 0.075, 0.050, 0.030, 0.070, 0.040, 0.060, 0.025)   # m, by seed % 8
EMPTY = 15e-6                                   # a bottle with 15 mL or less left counts as emptied
SPILL_LIMIT = 10e-6                             # more than 10 mL outside the tank: spilled
EDGE = 0.005                                    # liquid must fall at least 5 mm inside the walls
DT = 0.05                                       # control step at 20 Hz (re-read from the env at reset)


def _cap_table(n: int = 120) -> np.ndarray:
    """Liquid (m^3) a bottle holds without pouring, for tilts of 0..180 degrees: the cylinder below the horizontal
    plane through the mouth's lowest point."""
    xs = (np.arange(n) + 0.5) / n * 2 * B_R - B_R
    x, y = np.meshgrid(xs, xs)
    disc, da = x * x + y * y <= B_R * B_R, (2 * B_R / n) ** 2
    out = []
    for deg in range(181):
        t = math.radians(deg)
        s, c = math.sin(t), math.cos(t)
        if abs(c) < 1e-9:
            length = np.where(x + MOUTH_R <= 0, B_H, 0.0)
        elif c > 0:
            length = np.clip(B_H - s * (x + MOUTH_R) / c, 0.0, B_H)
        else:
            length = B_H - np.clip(B_H + s * (x + MOUTH_R) / -c, 0.0, B_H)
        out.append(float((length * disc).sum() * da))
    return np.array(out)


CAP = _cap_table()


def capacity(tilt: float) -> float:
    return float(np.interp(math.degrees(tilt), np.arange(181), CAP))


def axis_of(state) -> np.ndarray:
    return tankkit.quat_to_matrix(state.quat_xyzw)[:, 2]


def tilt_of(state) -> float:
    return math.acos(max(-1.0, min(1.0, float(axis_of(state)[2]))))


def lip(state) -> np.ndarray:
    """World point the liquid leaves from: the mouth's lowest point."""
    a = axis_of(state)
    centre = np.asarray(state.pos, float) + a * B_H / 2
    down = np.array([0.0, 0.0, -1.0]) - a * (-a[2])
    n = np.linalg.norm(down)
    return centre + (MOUTH_R * down / n if n > 1e-6 else 0.0)


def over_tank(p) -> bool:
    return bool(abs(p[0] - TX) < W / 2 - EDGE and abs(p[1] - TY) < W / 2 - EDGE and p[2] > TOP + FLOOR)


def valve_open(angle: float) -> float:
    """Fraction of the full flow the valve passes at lever ``angle`` (rad): 1 open, falling to 0 at 85 degrees."""
    return float(np.clip(1.0 - angle / CLOSED_AT, 0.0, 1.0))


def start_level(seed) -> float:
    return START_LEVELS[int(seed if seed is not None else os.environ.get("TANK_SEED", 0)) % len(START_LEVELS)]


# -- run-time state --------------------------------------------------------------------------------------------------
_STATE: dict = {}


def _reset_state(environment=None) -> None:
    global DT
    level = start_level(tankkit.SEED)
    _STATE.update(t=0.0, volume=level * AREA, bottles={n: FILL for n in BOTTLE_NAMES}, poured={n: 0.0 for n in
                  BOTTLE_NAMES}, spilled=0.0, overflow=False, in_tank=False, moved=False, dropped=False,
                  lamp=False, alarm_t=None, closed_t=None, shown=None, stream=None, start=level)
    if environment is not None:
        DT = float(getattr(environment.unwrapped, "step_dt", DT))
        _show(environment)


def level() -> float:
    return _STATE["volume"] / AREA


def _show(environment) -> None:
    """Draw the liquid to the nearest 5 mm layer, the lamp and the inlet's stream."""
    n = min(N_LAYERS, int(round(level() / LAYER)))
    if n != _STATE["shown"]:
        set_visible(environment, "tank", range(FIRST_LAYER, FIRST_LAYER + n), True)
        set_visible(environment, "tank", range(FIRST_LAYER + n, FIRST_LAYER + N_LAYERS), False)
        _STATE["shown"] = n
    set_visible(environment, "tank", (LAMP_LIT,), _STATE["lamp"])
    stream = _STATE.get("flow", 0.0) > 0
    if stream != _STATE["stream"]:
        set_visible(environment, "tank", (STREAM,), stream)
        _STATE["stream"] = stream


def advance(objects, dt: float) -> None:
    """One control step of liquid: the inlet and each bottle's outflow, into the tank or onto the table."""
    tank = objects["tank"]
    _STATE["t"] += dt
    flow = INFLOW * valve_open(tank.hinge)
    _STATE["flow"] = flow
    _STATE["volume"] += flow * dt
    if flow == 0 and _STATE["closed_t"] is None:
        _STATE["closed_t"] = _STATE["t"]
    for n in BOTTLE_NAMES:
        b = objects[n]
        excess = _STATE["bottles"][n] - capacity(tilt_of(b))
        if excess > 0:
            out = min(excess, POUR_MAX * dt)
            _STATE["bottles"][n] -= out
            if over_tank(lip(b)):
                _STATE["volume"] += out
                _STATE["poured"][n] += out
            else:
                _STATE["spilled"] += out
        inside = abs(b.pos[0] - TX) < W / 2 and abs(b.pos[1] - TY) < W / 2 and b.pos[2] < TOP + H
        _STATE["in_tank"] |= bool(inside)
        _STATE["dropped"] |= bool(b.pos[2] < TOP - 0.05)
    _STATE["overflow"] |= level() > RIM
    lit = level() >= LINE or (_STATE["lamp"] and level() >= LINE - 0.002)
    _STATE["lamp"] = bool(lit)
    if lit and _STATE["alarm_t"] is None:
        _STATE["alarm_t"] = _STATE["t"]
    r = tankkit.quat_to_matrix(tank.quat_xyzw)
    _STATE["moved"] |= bool(np.linalg.norm(np.asarray(tank.pos)[:2] - (TX, TY)) > 0.02 or r[2, 2] < math.cos(
        math.radians(5)))


_reset_state()
on_reset(_reset_state)


@on_step
def _liquid(environment) -> None:
    from yam_arena.scene import object_states

    objects = object_states(environment, ["tank", *BOTTLE_NAMES])
    advance(objects, DT)
    _show(environment)
    if os.environ.get("TANK_DEBUG") and round(_STATE["t"] / DT) % 20 == 0:
        print(f"[tank] t={_STATE['t']:.1f} level={100 * level():.2f}cm lamp={_STATE['lamp']} "
              f"flow={_STATE['flow'] * 6e4:.2f}L/min bottles=" +
              ",".join(f"{1e6 * v:.0f}" for v in _STATE["bottles"].values()) +
              f" spilled={1e6 * _STATE['spilled']:.1f}mL score={_score(None):.2f} lever="
              f"{math.degrees(objects['tank'].hinge):.0f} at=" + " ".join(
                  f"({100 * objects[n].pos[0]:.0f},{100 * objects[n].pos[1]:.0f},{100 * (objects[n].pos[2] - TOP):.0f}"
                  f"/{math.degrees(tilt_of(objects[n])):.0f})" for n in BOTTLE_NAMES), flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------
def emptied() -> int:
    return sum(_STATE["bottles"][n] <= EMPTY for n in BOTTLE_NAMES)


def _score(tank) -> float:
    if tank is not None:
        _STATE["lever"] = float(tank.hinge)
    if (_STATE["overflow"] or _STATE["spilled"] > SPILL_LIMIT or _STATE["in_tank"] or _STATE["moved"]
            or _STATE["dropped"]):
        return 0.0
    closed = _STATE.get("lever", 0.0) >= CLOSED_AT
    at_line = level() >= LINE
    if at_line and not closed:
        return 0.0  # the alarm is on and the inlet still runs: it will overflow
    return round(0.15 * emptied() + (0.4 if closed and at_line else 0.0), 6)


def success(objects) -> bool:
    """All four bottles emptied into the tank, the level at or above the fill line, the inlet shut, nothing
    overflowed or spilled; held 1 s."""
    return _score(objects["tank"]) >= 1.0 - 1e-9


def score(objects) -> float:
    """0 if the tank ever overflowed, more than 10 mL was spilled, a bottle went into the tank or off the table, or
    the tank was moved 2 cm or tipped 5 degrees; 0 too if the level is at the line with the inlet still open.
    Otherwise 0.15 per bottle emptied into the tank, and 0.4 more for the inlet shut with the level at the line."""
    return _score(objects["tank"])


# -- scripted demo -----------------------------------------------------------------------------------------------------
OPEN, CLOSED = -0.04695, 0.0                    # finger joint targets (m)
SQUEEZE = -0.028                                # pads commanded to 56 mm on the 70 mm bottle
TIP = 0.013


def frame(z, x) -> np.ndarray:
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE = frame((1, 0, 0), (0, -1, 0))             # approach forward, fingers closing along y: grips an upright bottle
GRIP_Z = 0.13                                   # TCP 13 cm up the bottle (5 cm below its top)
CARRY_Z = TOP + 0.35                            # the bottle's bottom 2 cm above the rim while carried
POUR_AT = (TX, 0.09, TOP + 0.33)
POUR_TILT = math.radians(float(os.environ.get("POUR_TILT", 118)))
POUR_HOLD = float(os.environ.get("POUR_HOLD", 6.0))


EMPTY_XY = [(BIN_XY[0] + dx, BIN_XY[1] + dy) for dx, dy in ((0.05, 0.06), (0.05, -0.06), (-0.05, 0.06), (-0.05, -0.06))]


def _pour(x, y, ex=None, ey=None):
    """One bottle: side grip, lift, carry over the tank, roll the wrist to tip it toward the middle, hold, and stand
    the empty at (ex, ey), clear of the full ones' approach."""
    ex, ey = (x, y) if ex is None else (ex, ey)
    grip = np.array([x, y, TOP + GRIP_Z])
    put = np.array([ex, ey, TOP + 0.19])         # the empty's base 6 cm above the table, inside the bin
    pre = grip - (0.07, 0, -0.01)
    up = np.array([x, y, CARRY_Z])
    over = np.array([POUR_AT[0], POUR_AT[1], CARRY_Z])
    pour = np.array(POUR_AT)
    tipped = rot_x(POUR_TILT) @ SIDE
    half = rot_x(POUR_TILT * 0.6) @ SIDE
    return [(2.5, (pre[0], pre[1], CARRY_Z), SIDE, OPEN, "to the bottle"), (1.2, pre, SIDE, OPEN, "behind the bottle"),
            (0.4, pre, SIDE, OPEN, "settle"), (1.2, grip, SIDE, OPEN, "fingers round it"),
            (0.8, grip, SIDE, SQUEEZE, "grip"), (1.4, up, SIDE, SQUEEZE, "lift"),
            (1.8, over, SIDE, SQUEEZE, "over the tank"), (0.8, pour, SIDE, SQUEEZE, "down to pour"),
            (1.5, pour, half, SQUEEZE, "tip"), (1.5, pour, tipped, SQUEEZE, "pour"),
            (POUR_HOLD, pour, tipped, SQUEEZE, "drain"), (1.5, pour, SIDE, SQUEEZE, "upright"),
            (0.8, over, SIDE, SQUEEZE, "up"), (2.0, (0.30, 0.30, CARRY_Z + 0.03), SIDE, SQUEEZE, "back over the table"),
            (2.0, (ex, ey, CARRY_Z), SIDE, SQUEEZE, "to the bin"),
            (1.2, put, SIDE, SQUEEZE, "into the bin"), (0.5, put, SIDE, SQUEEZE, "settle"),
            (0.6, put, SIDE, OPEN, "let go"), (1.0, (ex, ey, CARRY_Z), SIDE, OPEN, "up")]


def left_plan():
    return [w for (x, y), (ex, ey) in zip(BOTTLE_XY, EMPTY_XY) for w in _pour(x, y, ex, ey)]


# The right arm turns the lever with its closed fingertips, pitched forward from straight down, pushing on the
# grip's trailing side as it swings from along the pipe (open) to across it (closed).
PUSH = rot_y(-math.radians(40)) @ frame((0, 0, -1), (1, 0, 0))
PIVOT_W = np.array([TX + PIVOT[0], TY + PIVOT[1], TOP + PIVOT[2]])
PUSH_R = LEVER_LEN - SLEEVE[1] / 2              # push on the middle of the grip
PUSH_Z = 0.006 + 0.003                          # fingertips 3 mm above the grip's mid-height


def _push_point(a: float) -> np.ndarray:
    d = np.array([math.sin(a), -math.cos(a), 0.0])
    t = np.array([math.cos(a), math.sin(a), 0.0])
    tips = PIVOT_W + PUSH_R * d - (SLEEVE[0] / 2 + 0.012) * t + (0, 0, PUSH_Z)
    return tips - TIP * PUSH[:, 2]


def right_wait():
    p = _push_point(0.0)
    return [(2.5, p + (0, 0, 0.07), PUSH, CLOSED, "wait by the valve")]


def right_close():
    p0 = _push_point(0.0)
    plan = [(1.0, p0 + (0, 0, 0.02), PUSH, CLOSED, "behind the grip"), (0.6, p0, PUSH, CLOSED, "down")]
    for k in range(1, 7):
        plan.append((0.45, _push_point(math.radians(106) * k / 6), PUSH, CLOSED, "turn the lever"))
    end = _push_point(math.radians(106))
    return plan + [(0.5, end, PUSH, CLOSED, "hold at the stop"), (0.8, end + (-0.05, 0, 0.06), PUSH, CLOSED, "clear"),
                   (1.5, p0 + (-0.04, 0, 0.10), PUSH, CLOSED, "back")]


def lamp_pixels(image) -> int:
    """Bright red pixels in a camera image: the lit alarm lamp (nothing else in the scene is saturated red)."""
    im = np.asarray(image).astype(np.int16)
    r, g, b = im[..., 0], im[..., 1], im[..., 2]
    return int(((r > 150) & (g < 90) & (b < 90) & (r - g > 90)).sum())


class Controller:
    """Closed loop on the top camera: the left arm pours the bottles one after another; the right arm waits by the
    valve and turns it shut once it has seen the lamp lit in 3 frames in a row. Cheats (DEMO_CHEAT): ``ignore``
    never shuts the valve, ``first`` shuts it before pouring, ``timed`` shuts it at a fixed 45 s."""

    def __init__(self, bases, dt, cheat=""):
        from yam_arena.demo import ArmPlan, Waypoint

        self.Waypoint, self.ArmPlan = Waypoint, ArmPlan
        self.dt, self.t, self.cheat = float(dt), 0.0, cheat
        self.bases = bases
        self.left = ArmPlan(self._wps(left_plan()), bases["left"])
        self.right = ArmPlan(self._wps(right_wait()), bases["right"])
        self.right_t0, self.closing, self.seen, self.end_t = 0.0, False, 0, None
        if cheat == "first":
            self._start_close()

    def _wps(self, plan):
        return [self.Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in plan]

    def _start_close(self):
        q = self.right.q.copy()
        self.right = self.ArmPlan(self._wps(right_close()), self.bases["right"], q_start=q, finger_start=CLOSED)
        self.right_t0, self.closing = self.t, True

    def act(self, observation):
        from inspect_robots_yam.packing import pack

        if not self.closing:
            seen = lamp_pixels(observation.images["top_cam"]) >= 12
            self.seen = self.seen + 1 if seen else 0
            if (self.cheat == "" and self.seen >= 3) or (self.cheat == "timed" and self.t >= 45.0):
                self._start_close()
        if self.cheat == "first":
            t_left = max(0.0, self.t - (self.right.duration + 0.5))
        else:
            t_left = self.t
        self.t += self.dt
        left = self.left.action7(t_left + self.dt)[0]
        right = self.right.action7(self.t - self.right_t0)[0]
        left_done = t_left > self.left.duration
        right_done = self.closing and self.t - self.right_t0 > self.right.duration
        if left_done and (right_done or self.cheat == "ignore") and self.end_t is None:
            self.end_t = self.t
        done = self.end_t is not None and self.t > self.end_t + (2.0 if self.cheat != "ignore" else 40.0)
        return pack(left, right), done


def demo_controller(bases, dt):
    return Controller(bases, dt, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
