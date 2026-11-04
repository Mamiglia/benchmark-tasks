"""On the station table: a clear glass jar (7 cm wide, 8.5 cm tall) with a red screw lid, a little left of centre.
The lid is half screwed on: one of its two turns of thread is left. Inside the jar stands a yellow wooden block
4 mm taller than the jar, so the lid sits on the block and cannot go further. Turning the lid on hard enough crushes
the block's top (damage). The robot must screw the lid fully onto the jar.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import (Attach, PartState, on_physics, on_reset, part, rot_z, set_attach_target,  # noqa: E402
                      set_attached, set_collision)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Screw the lid fully onto the jar."

# The jar's walls and floor collide but are not drawn (RTX in this setup renders partial opacity as grey, which
# hid the block); thin light-blue edges inside the wall's thickness (rim, base, bead, eight posts) show the glass.
CLEAR, EDGE = (1.0, 1.0, 1.0, 0.0), (0.70, 0.82, 0.92)
RED, YELLOW = (0.80, 0.08, 0.08), (1.0, 0.70, 0.0)
SIDES = 16  # jar and lid are 16-sided: round enough to turn inside each other


def ring(a_in, a_out, z0, z1, color, sides=SIDES):
    """A 16-sided ring (or, with a_in = 0, a disc) between apothems ``a_in`` and ``a_out`` (m), from z0 to z1."""
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    return [(((a_in + a_out) / 2 * math.cos(t), (a_in + a_out) / 2 * math.sin(t), (z0 + z1) / 2),
             (a_out - a_in, side, z1 - z0), color, math.degrees(t))
            for t in (2 * math.pi * k / sides for k in range(sides))]


# -- the jar (origin: centre of its base) --------------------------------------------------------------------------
JX, JY = 0.26, 0.04
J_OUT, J_WALL, J_FLOOR = 0.034, 0.003, 0.004  # 6.8 cm across the flats, 3 mm wall, 4 mm floor
RIM = 0.085                                    # the rim: 8.5 cm above the table; the lid's seat
PITCH, TURNS = 0.004, 2.0                      # 4 mm per turn, two turns of thread
TRAVEL = PITCH * TURNS                         # the lid's 8 mm travel from first catch to seated
J_IN = J_OUT - J_WALL
BEAD_TOP = RIM - TRAVEL - 0.001                # a glass bead below the thread: a seated lid's skirt ends 1 mm above it
POST_W = 0.001
JAR = part("jar", pos=(JX, JY, TOP), mass=0.30, color=EDGE, friction=0.5,
           boxes=ring(0, J_OUT, 0, J_FLOOR, CLEAR) + ring(J_IN, J_OUT, J_FLOOR, RIM, CLEAR)
           + ring(J_OUT, J_OUT + 0.003, BEAD_TOP - 0.002, BEAD_TOP, EDGE)
           + ring(J_IN, J_OUT, RIM - 0.002, RIM, EDGE) + ring(J_IN, J_OUT, 0, 0.003, EDGE)
           + [(((J_IN + J_OUT) / 2 * math.cos(t), (J_IN + J_OUT) / 2 * math.sin(t), RIM / 2),
               (J_WALL, POST_W, RIM), EDGE, math.degrees(t)) for t in (math.pi * k / 4 for k in range(8))])

# -- the lid (origin: centre of its top plate's underside; on the rim when seated) ----------------------------------
# Its skirt reaches 8 mm down. The skirt's inner part overlaps the jar's wall by 3 mm, so a lid set on the jar rests
# on the rim with the skirt at the thread's start. Threaded, that inner part's collision is switched off and the
# skirt slides over the glass (where the real thread is); the outer part, which the gripper holds, never overlaps.
L_OUT, L_IN, L_TOP = 0.037, 0.031, 0.003
L_MID = J_OUT / math.cos(math.pi / SIDES) + 0.001  # clear of the jar wall's corners at any angle
H0 = TRAVEL - PITCH                            # start: one turn in, one turn to go (4 mm above seated)
K_ROT, K_LIN = 6.0, 20000.0                    # N m/rad, N/m: thread drives (see "Mechanisms" in the README)
SCREW = Attach("jar", pos0=(0.0, 0.0, RIM), stiffness=K_ROT, damping=0.02, travel=TRAVEL + 0.002,
               lin_stiffness=K_LIN, lin_damping=100.0, lin_max_force=40.0)
LID = part("lid", pos=(JX, JY, TOP + RIM + H0), mass=0.03, color=RED, friction=1.0, attach=SCREW,
           boxes=ring(0, L_OUT, 0, L_TOP, RED) + ring(L_MID, L_OUT, -TRAVEL, 0, RED) + ring(L_IN, L_MID, -TRAVEL, 0, RED))
INNER_SKIRT = range(2 * SIDES, 3 * SIDES)       # indices of the inner skirt's boxes in LID.boxes

# -- the block: wood, 2.4 x 2.4 cm, its top 4 mm above the rim -------------------------------------------------------
BLOCK_W = 0.024
BLOCK_H = RIM + H0 - J_FLOOR                   # 8.5 cm
# Leaving the tipped jar it pivots over the 2 mm rim at up to 0.3 m/s (1.25 mm per physics step). With 1 mm contact
# offsets a corner got into the glass before a contact existed and was pushed out at the 1 m/s depenetration cap:
# it shot 13 cm sideways spinning at 40 rad/s. A 5 mm offset and speculative CCD make the contact in time.
BLOCK = part("block", pos=(JX, JY, TOP + J_FLOOR), mass=0.03, color=YELLOW, friction=0.3, contact_offset=0.005, ccd=True,
             boxes=[((0, 0, BLOCK_H / 2), (BLOCK_W, BLOCK_W, BLOCK_H), YELLOW)])

# -- the tray: an open plastic tray under where the tipped jar's mouth points; the block may go in it (optional) -----
TX, TY = 0.11, 0.07
T_IN_X, T_IN_Y, T_WALL, T_FLOOR, T_H = 0.16, 0.20, 0.005, 0.003, 0.03
BLUE = (0.15, 0.30, 0.60)
TRAY = part("tray", pos=(TX, TY, TOP), mass=0.15, color=BLUE, friction=0.6,
            boxes=[((0, 0, T_FLOOR / 2), (T_IN_X + 2 * T_WALL, T_IN_Y + 2 * T_WALL, T_FLOOR), BLUE)]
            + [((sx * (T_IN_X + T_WALL) / 2, 0, T_H / 2), (T_WALL, T_IN_Y + 2 * T_WALL, T_H), BLUE) for sx in (-1, 1)]
            + [((0, sy * (T_IN_Y + T_WALL) / 2, T_H / 2), (T_IN_X, T_WALL, T_H), BLUE) for sy in (-1, 1)])
OBJECTS = [JAR, LID, BLOCK, TRAY]

# -- the thread (pure Python; the physics hook below feeds it) ------------------------------------------------------
# n: the lid's net tightening turn (rad, clockwise seen from above), 0 at the first catch, 2 TURNS pi seated.
# h: the lid's height above its seat (m). The thread wants h = PITCH * (TURNS - n / 2 pi).
FRICTION = 0.20                 # N m: running torque of the thread (breakaway friction, as in a real lid)
SLACK = math.radians(30)        # give: the lid can be turned 30 degrees past where its height says it should be
BREAK_NM = 0.6                  # torque on a blocked lid that crushes the block's top / strips the thread ...
BREAK_STEPS = 3                 # ... sustained this many physics steps (12.5 ms at 240 Hz)
SEATED = 0.001                  # m: a lid within 1 mm of its seat is seated (tightening it harder is not damage)
RELEASE = math.radians(10)      # unscrewed 10 degrees past the first catch: the lid is off the thread
SETTLE = 60                     # physics steps (0.25 s) after a catch in which the lid settles on the thread; no damage


def thread_height(n: float) -> float:
    """Below the first catch (n < 0) the lid keeps rising to RELEASE, so it leaves the thread clear of the rim."""
    return PITCH * (TURNS - min(max(n, -RELEASE), 2 * math.pi * TURNS) / (2 * math.pi))


class Thread:
    """The thread's state and per-physics-step targets. ``step(n, h)`` returns the turn drive's target (n units) and
    the slide drive's target height, and updates the damage latch."""

    def __init__(self):
        self.reset(2 * math.pi * (TURNS - H0 / PITCH))

    def reset(self, n: float) -> None:
        self.anchor = n
        self.over = 0
        self.peak_nm = 0.0      # largest torque against a blocked (unseated) lid
        self.damaged = False
        self.settle = SETTLE

    def step(self, n: float, h: float) -> tuple[float, float]:
        # breakaway friction: a stiff spring to an anchor that slips once it pulls with more than FRICTION
        pull = K_ROT * (self.anchor - n)
        if abs(pull) > FRICTION:
            self.anchor = n + math.copysign(FRICTION, pull) / K_ROT
        # the lid cannot turn further on than its actual height allows: past that it pushes back, stiffly
        n_max = 2 * math.pi * (TURNS - h / PITCH) + SLACK
        self.anchor = min(self.anchor, n_max)
        jam = K_ROT * (n - n_max)
        if self.settle > 0:
            self.settle -= 1
            return self.anchor, thread_height(n)
        if h > SEATED:
            self.peak_nm = max(self.peak_nm, jam)
        self.over = self.over + 1 if (h > SEATED and jam > BREAK_NM) else 0
        self.damaged |= self.over >= BREAK_STEPS
        return self.anchor, thread_height(n)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _twist(jar: PartState, lid: PartState) -> float:
    """The lid's turn about the jar's axis, relative to the jar (rad, counterclockwise from above +)."""
    r = jar.rotation().T @ lid.rotation()
    return math.atan2(r[1, 0], r[0, 0])


def _height(jar: PartState, lid: PartState) -> float:
    return float(jar.to_local(lid.pos)[2] - RIM)


def can_catch(jar: PartState, lid: PartState) -> bool:
    """A lid set on the rim catches the thread: centred within 6 mm, tilted under 10 degrees, its skirt within 3 mm of
    the thread's start, and nearly still."""
    p = jar.to_local(lid.pos)
    tilt = math.acos(max(-1.0, min(1.0, float((jar.rotation().T @ lid.rotation())[2, 2]))))
    return bool(math.hypot(p[0], p[1]) < 0.006 and tilt < math.radians(10) and abs(p[2] - RIM - TRAVEL) < 0.003
                and np.linalg.norm(lid.lin_vel - jar.lin_vel) < 0.05)


THREAD = Thread()
_STATE: dict = {}
_DEBUG = [0] if os.environ.get("JAR_DEBUG") else []


def _reset_state(environment=None) -> None:
    n0 = 2 * math.pi * (TURNS - H0 / PITCH)
    THREAD.reset(n0)
    _STATE.update(engaged=True, n=n0, twist=None, lid_off=False, knocked=False, catches=0)
    if environment is not None:
        set_attached(environment, "lid", True)
        set_collision(environment, "lid", INNER_SKIRT, False)
        set_attach_target(environment, "lid", 0.0, H0)


_reset_state()
on_reset(_reset_state)


@on_physics
def _thread(environment) -> None:
    """Every physics step: count the lid's turn, set the thread's drive targets, release and catch the lid."""
    from yam_arena.scene import object_states

    s = object_states(environment, ["jar", "lid"])
    jar, lid = s["jar"], s["lid"]
    twist = _twist(jar, lid)
    if not _STATE["engaged"]:
        # off once it has been 2 cm from its seat; only then can it catch the thread again
        _STATE["lid_off"] |= bool(np.linalg.norm(lid.pos - jar.to_world((0.0, 0.0, RIM))) > LID_AWAY)
        if _DEBUG:
            _DEBUG[0] += 1
            if _DEBUG[0] % 24 == 0:
                q = jar.to_local(lid.pos)
                print(f"[jar] {_DEBUG[0] // 12:5d} jar {np.round(jar.pos[:2] * 1000, 1)} free lid at "
                      f"{np.round(q * 1000, 1)} mm (jar frame), tilt "
                      f"{math.degrees(lid.tilt()):.1f} jar tilt {math.degrees(jar.tilt()):.1f}", flush=True)
        if _STATE["lid_off"] and can_catch(jar, lid):
            THREAD.reset(0.0)
            _STATE.update(engaged=True, n=0.0, twist=twist, catches=_STATE["catches"] + 1)
            set_attach_target(environment, "lid", twist, TRAVEL)
            set_attached(environment, "lid", True)
            set_collision(environment, "lid", INNER_SKIRT, False)
        return
    if _STATE["twist"] is not None:
        _STATE["n"] -= _wrap(twist - _STATE["twist"])  # clockwise tightens
    _STATE["twist"] = twist
    n, h = _STATE["n"], _height(jar, lid)
    target, height = THREAD.step(n, h)
    set_attach_target(environment, "lid", _wrap(twist - (target - n)), height)
    if n < -RELEASE:
        set_attached(environment, "lid", False)  # off the thread: a free lid resting on the rim
        set_collision(environment, "lid", INNER_SKIRT, True)
        _STATE["engaged"] = False
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 24 == 0:
            print(f"[jar] {_DEBUG[0] // 12:5d} jar {np.round(jar.pos[:2] * 1000, 1)} n {math.degrees(n):7.1f} deg  h {h * 1000:5.2f} mm  target "
                  f"{math.degrees(target):7.1f}  peak {THREAD.peak_nm:.2f} N m  damaged {THREAD.damaged}  "
                  f"engaged {_STATE['engaged']}  catches {_STATE['catches']}", flush=True)


_CORNERS = np.array([(sx * BLOCK_W / 2, sy * BLOCK_W / 2, z) for sx in (-1, 1) for sy in (-1, 1)
                     for z in (0.0, BLOCK_H)])
_NORMALS = np.array([(math.cos(2 * math.pi * k / SIDES), math.sin(2 * math.pi * k / SIDES)) for k in range(SIDES)])


def block_overlap(jar: PartState, block: PartState) -> float:
    """How deep (m) the block's deepest corner is inside the jar's wall or floor (0 if none is)."""
    p = np.array([jar.to_local(block.to_world(c)) for c in _CORNERS])
    r = (p[:, :2] @ _NORMALS.T).max(axis=1)  # 16-sided: distance out along the nearest face normal
    inside = (r < J_OUT + 0.01) & (p[:, 2] > 0) & (p[:, 2] < RIM)  # also a corner pushed right through the wall
    wall = np.where(inside, r - J_IN, 0.0)
    floor = np.where(inside & (r < J_IN), J_FLOOR - p[:, 2], 0.0)
    return float(max(0.0, np.minimum(np.maximum(wall, floor), J_WALL).max()))


if _DEBUG:
    _BLOCK_STEPS = [0]

    @on_physics
    def _watch_block(environment) -> None:
        from yam_arena.scene import object_states

        s = object_states(environment, ["jar", "block"])
        jar, block = s["jar"], s["block"]
        d = block_overlap(jar, block)
        _BLOCK_STEPS[0] += 1
        if _BLOCK_STEPS[0] % 12 == 0 or d > 0.0003 or np.linalg.norm(block.lin_vel) > 0.3:
            q = jar.to_local(block.pos)
            print(f"[block] {_BLOCK_STEPS[0]:6d} overlap {d * 1000:4.1f} mm  block in jar {np.round(q * 1000, 1)} "
                  f"tilt {math.degrees(block.tilt()):5.1f} jar tilt {math.degrees(jar.tilt()):5.1f} "
                  f"jar z {jar.pos[2] * 1000:6.1f} v {np.round(block.lin_vel, 2)} w {np.linalg.norm(block.ang_vel):.1f} "
                  f"at {np.round(block.pos * 1000)}", flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------
LID_AWAY = 0.02                  # the lid is off once it has been 2 cm from its seat, off the thread
THREADED = PITCH * (TURNS - 0.5)  # back on and turned at least half a turn: 6 mm or less above its seat
KNOCK_TILT, FALL = math.radians(45), 0.05


def block_out(jar: PartState, block: PartState) -> bool:
    """The block's centre is outside the jar's inside (beyond its inner wall, or above the rim)."""
    p = jar.to_local(block.pos)
    return bool(math.hypot(p[0], p[1]) > J_IN or not 0 < p[2] < RIM)


def _update(objects) -> float:
    jar, lid, block = objects["jar"], objects["lid"], objects["block"]
    h = _height(jar, lid)
    # knocked over: tilted past 45 degrees while touching the table (a jar tipped in the hand is lifted)
    _STATE["knocked"] |= bool(jar.tilt() > KNOCK_TILT and jar.low[2] < TOP + 0.01) or jar.pos[2] < TOP - FALL
    if THREAD.damaged or _STATE["knocked"] or not _STATE["lid_off"]:
        return 0.0
    if not block_out(jar, block):
        return 0.25
    if _STATE["engaged"] and h <= SEATED:
        return 1.0
    if _STATE["engaged"] and h <= THREADED:
        return 0.75
    return 0.5


def success(objects) -> bool:
    """Lid seated (within 1 mm) after being taken off, the block out of the jar, nothing damaged; held 1 s."""
    return _update(objects) == 1.0


def score(objects) -> float:
    """0 if the blocked lid was forced (0.6 N m for 12.5 ms), the jar was knocked over or fell, or the lid was never
    taken off. Otherwise 0.25 lid off; 0.5 and the block out of the jar; 0.75 and the lid back on, at least half a
    turn down (6 mm or less above its seat); 1.0 and the lid seated (1 mm or less)."""
    return _update(objects)


# -- scripted demo (pure NumPy plans; demo() wraps them in yam_arena Waypoints) -------------------------------------
OPEN = -0.04695                   # finger joint target (m): yam_arena.constants.FINGER_OPEN, pads 94 mm apart
# Grip targets 3 mm inside the part's faces: each finger presses with its spring (about 15 N) and the two centre the
# part between them. Closing fully would saturate both fingers at their effort limit, and the gripped part could
# then slide along the closing axis.
JAR_PINCH, LID_PINCH = -(J_OUT - 0.003), -(L_OUT - 0.003)


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def down(yaw: float) -> np.ndarray:
    """Straight-down grip turned ``yaw`` about the vertical (yam_arena.demo.grip_rotation)."""
    return rot_z(yaw) @ frame((0, 0, -1), (0, -1, 0))


# Left arm: pinches the jar's front and back 4.5 cm up, from its left, pitched 20 degrees down.
HOLD = frame((0, -math.cos(math.radians(20)), -math.sin(math.radians(20))), (-1, 0, 0))
GRASP = (JX, JY, TOP + 0.045)
TIP = -2.2                         # 126 degrees about the world y axis: the jar's mouth swings back and down
TIP_AT = (JX - 0.06, JY - 0.04, TOP + 0.24)  # the mouth over the tray; the block tends to fly a few cm to +y
LID_AT = (0.20, -0.15)             # where the right arm sets the lid down on the table
STROKE = math.pi                   # one 180-degree turn of the wrist per grip
PAD = 0.002 + 0.013                # pads 2 mm above the skirt's lower edge; TCP 13 mm above the pad tips


def _skirt_z(h: float) -> float:
    """TCP height for pinching the lid's skirt with the lid ``h`` above its seat."""
    return TOP + RIM + h - TRAVEL + PAD


def _strokes(k: int, h0: float, sign: int, last_extra: float = 0.0):
    """``k`` turns of 180 degrees (sign +1: counterclockwise, unscrewing), the hand following the thread up or down
    2 mm per turn; between turns it lets go, rises 1.5 cm, and turns back."""
    at = (JX, JY)
    a0, a1 = (-math.pi / 4, 3 * math.pi / 4) if sign > 0 else (3 * math.pi / 4, -math.pi / 4)
    plan, h = [], h0
    for i in range(k):
        end = a1 + (-sign * last_extra if i == k - 1 else 0.0)
        h1 = h + sign * PITCH / 2
        z0, z1 = _skirt_z(h), _skirt_z(h1)
        plan += [(1.0, (*at, z0 + 0.015), down(a0), OPEN, "over the lid"), (0.8, (*at, z0), down(a0), OPEN, "down"),
                 (0.8, (*at, z0), down(a0), LID_PINCH, "pinch"),
                 (1.25, (*at, (z0 + z1) / 2), down((a0 + end) / 2), LID_PINCH, "turn"),
                 (1.25, (*at, z1), down(end), LID_PINCH, "turn"), (0.6, (*at, z1), down(end), OPEN, "let go"),
                 (0.6, (*at, z1 + 0.015), down(end), OPEN, "up")]
        h = h1
    return plan


def _right_open():
    """Unscrew (three turns of 180 degrees; the lid comes off the thread during the third), lift the lid off and set it
    down on the table."""
    lift = (JX, JY, _skirt_z(TRAVEL) + 0.05)
    place = (*LID_AT, TOP + PAD + 0.001)
    return ([(3.0, (JX, JY, _skirt_z(H0) + 0.04), down(-math.pi / 4), OPEN, "over the lid")]
            + _strokes(3, H0, +1)[:-2]
            + [(1.2, lift, down(3 * math.pi / 4), LID_PINCH, "lift the lid off"),
               (2.0, (*LID_AT, TOP + 0.06), down(3 * math.pi / 4), LID_PINCH, "carry it aside"),
               (1.0, place, down(3 * math.pi / 4), LID_PINCH, "set it down"),
               (0.6, place, down(3 * math.pi / 4), OPEN, "let go"),
               (1.0, (*LID_AT, TOP + 0.10), down(3 * math.pi / 4), OPEN, "clear")])


def _right_close():
    """Pick the lid up, set it on the rim, and screw it on: four turns of 180 degrees (two turns), the last one 20
    degrees longer to take up slip."""
    pick = (*LID_AT, TOP + PAD + 0.001)
    over = (JX, JY, _skirt_z(TRAVEL) + 0.03)
    rest = (JX, JY, _skirt_z(TRAVEL) + 0.002)
    yaw = 3 * math.pi / 4
    return ([(1.0, pick, down(yaw), OPEN, "down to the lid"), (0.8, pick, down(yaw), LID_PINCH, "pick it up"),
             (1.2, (*LID_AT, TOP + 0.08), down(yaw), LID_PINCH, "lift"),
             (2.0, over, down(yaw), LID_PINCH, "over the jar"),
             (1.2, rest, down(yaw), LID_PINCH, "set it on the rim"), (0.6, rest, down(yaw), OPEN, "let go"),
             (0.8, (JX, JY, rest[2] + 0.015), down(yaw), OPEN, "up")]
            + _strokes(4, TRAVEL, -1, last_extra=math.radians(20))
            + [(1.0, (JX, JY, TOP + 0.12), down(-math.pi / 4), OPEN, "clear")])


def _right_force():
    """DEMO_FORCE: pinch the half-screwed lid and keep turning it on (clockwise)."""
    return ([(3.0, (JX, JY, _skirt_z(H0) + 0.04), down(3 * math.pi / 4), OPEN, "over the lid")]
            + _strokes(2, H0, -1)
            + [(1.0, (JX, JY, TOP + 0.12), down(-math.pi / 4), OPEN, "clear")])


def _left(hold_open: float, hold_close: float):
    """Grip the jar; hold it while the lid comes off; lift it, tip the block out, set it back; hold it while the lid
    goes on; let go."""
    pre = (JX, JY + 0.07, TOP + 0.07)
    up = (JX, JY, TOP + 0.22)
    plan = [(2.0, pre, HOLD, OPEN, "to the jar"), (1.5, GRASP, HOLD, OPEN, "around the jar"),
            (0.8, GRASP, HOLD, JAR_PINCH, "grip"), (hold_open, GRASP, HOLD, JAR_PINCH, "hold the jar")]
    if hold_close is None:
        return plan
    # The left arm's tracking error at this pose with the jar in hand, measured in simulation: commanding GRASP puts
    # the jar 6.8 mm short in x. The first grip doesn't show it (the compliant arm gives way to the standing jar).
    set_back = (JX + 0.0068, JY - 0.0018, GRASP[2] + 0.002)
    regrip = (set_back[0], set_back[1], GRASP[2])
    return plan + [(2.0, up, HOLD, JAR_PINCH, "lift"), (2.5, TIP_AT, rot_y(TIP) @ HOLD, JAR_PINCH, "tip it over"),
                   (2.5, TIP_AT, rot_y(TIP) @ HOLD, JAR_PINCH, "the block falls out"),
                   (2.5, up, HOLD, JAR_PINCH, "upright"), (2.0, set_back, HOLD, JAR_PINCH, "set it back"),
                   (0.4, set_back, HOLD, JAR_PINCH - 0.008, "loosen"),  # the fingers re-centre the jar
                   (0.6, regrip, HOLD, JAR_PINCH, "grip again"),
                   (hold_close, regrip, HOLD, JAR_PINCH, "hold the jar"), (0.6, regrip, HOLD, OPEN, "let go"),
                   (1.0, pre, HOLD, OPEN, "away")]


def plan(route: str = "demo") -> dict[str, list]:
    gripped = 2.0 + 1.5 + 0.8
    if route == "force":
        right = _right_force()
        return {"left": _left(sum(w[0] for w in right) - gripped, None), "right": right}
    opening, closing = _right_open(), _right_close()
    left = _left(sum(w[0] for w in opening) - gripped, 0.0)
    tipping = sum(w[0] for w in left[4:11])
    left = _left(sum(w[0] for w in opening) - gripped, sum(w[0] for w in closing[:-2]))
    right = opening + [(tipping, opening[-1][1], opening[-1][2], OPEN, "wait for the jar")] + closing
    return {"left": left, "right": right}


def demo():
    """Left arm grips the jar. Right arm unscrews the lid one turn plus (it comes off the thread), sets it on the
    table. Left arm lifts the jar, tips the block out onto the table, sets the jar back and holds it. Right arm puts
    the lid back on the rim and screws it two turns, down to its seat. DEMO_FORCE=1 instead keeps turning the
    half-screwed lid on: the blocked lid is forced and the trial scores 0."""
    from yam_arena.demo import Waypoint

    route = "force" if os.environ.get("DEMO_FORCE") else "demo"
    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in plan(route).items()}
