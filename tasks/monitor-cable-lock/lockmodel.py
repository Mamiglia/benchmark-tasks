"""lockmodel: geometry, the lock's key rules and the score bookkeeping of the monitor-lock task. NumPy only.

Frames (metres). World: x forward from the arms, y left, z up. The monitor's screen faces away from the robot (+x);
its back, with the security slot, faces the robot.

Lock frame H: origin at the centre of the lock head's front face (the face that sits flat on the monitor), x along
the lock's axis toward the T-bar (into the slot), y along the T-bar's crossbar when the key is in the unlocked
position, z = x cross y. Inserted, H's x is world +x and its y world y. The T-bar turns with the key about H's x;
turned +90 degrees (locked) its crossbar lies along H's z, across the slot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TOP = 0.7475  # station table top

# -- monitor (Dell E2422H, 23.8 in, on its fixed-height tilt stand; positions in world m) ----------------------------
PANEL_W, PANEL_H, DEPTH = 0.5526, 0.3316, 0.0497  # without stand (Dell user guide, "Physical characteristics")
PB = TOP + 0.0887  # panel's lower edge: 420.35 mm overall height minus the 331.61 mm panel, tilt 0
MON_Y = -0.070  # monitor centre line (y)
BACK_X = 0.420  # back face of the lower rear housing, where the slot is
SCREEN_X = BACK_X + DEPTH  # screen face (it faces away from the robot)
SLAB_X0 = BACK_X + 0.020  # back of the thin panel above and beside the rear housing
HOUSING = (MON_Y - 0.165, MON_Y + 0.165, PB, PB + 0.100)  # lower rear housing: y range, z range
MASS = 3.9  # kg, monitor with stand, no cables (guide: 4.03 kg with stand and cables)

# -- the security slot (Kensington Security Slot: 3 x 7 mm) in the rear housing, 115 mm from the centre line toward
# the robot's left and 45 mm above the panel's lower edge (guide, "Back view", item 1) -----------------------------
SLOT_Y, SLOT_Z = MON_Y + 0.115, PB + 0.045
SLOT_W, SLOT_H = 0.0070, 0.0030  # opening: wide along y, low along z
WALL = 0.0015  # wall the slot is cut in
CAVITY = (0.012, 0.012, 0.010)  # pocket behind the wall: y, z, depth (x)

# -- lock head and key (Kensington MicroSaver 2.0, K65020WW) ---------------------------------------------------------
NOSE = 0.004  # 10 mm front nose (the part that touches the monitor), this long
BARREL_L, BARREL_D = 0.030, 0.018  # barrel behind the nose
BACK = -(NOSE + BARREL_L)  # head's back face (H x), where the key goes in
PIN_X = 0.0022  # fixed side pins and the T-bar's stem stick out this far past the face
PIN_Y0, PIN_Y1, PIN_Z = 0.0014, 0.0029, 0.0022  # fixed pins: y from .. to (each side), height
STEM = 0.0020  # T-bar stem, square
BAR_X0, BAR_X1 = PIN_X, 0.0040  # crossbar: from the pins' end to the tip
BAR_TIP = 0.0034  # the crossbar's tip (its last 0.6 mm) is rounded off: narrower
BAR_L, BAR_T = 0.0060, 0.0022  # crossbar length (along H y, unlocked) and thickness
HEAD_MASS, TBAR_MASS = 0.050, 0.005
KEY_MASS = 0.008
SHANK = (BACK - 0.008, 0.007)  # exposed tubular shank: from x .. to BACK, 7 mm across
BOW = (SHANK[0] - 0.022, SHANK[0], 0.022, 0.003)  # key bow: x from .. to, width (along H y unlocked), thickness
KEY_GRIP_X = (BOW[0] + BOW[1]) / 2 - 0.003  # where the key hand holds the bow (H x)
HEAD_GRIP_X = BACK + 0.018  # where the head hand holds the barrel (H x)

# -- key and T-bar behaviour -----------------------------------------------------------------------------------------
LOCKED_ANGLE = math.pi / 2  # the key turns +90 degrees about H x to lock
DETENT_K, DETENT_D = 0.06, 0.002  # N m/rad, N m s/rad: weak detents at 0 and 90 degrees (real turn about 0.05 N m)
KEY_FREE_ANGLE = math.radians(80.0)  # the key comes out only in the locked position
KEY_PULL = 3.0  # N: the pull that takes a turned key out (its pins drag in the cylinder)
KEY_K = 2000.0  # N/m: in the locked position the key sits on this spring; ...
KEY_GIVE = KEY_PULL / KEY_K  # ... pulled out this far (1.5 mm, i.e. with 3 N), it comes out
CORD_PULL = 0.6  # N: the cable's weight and drag on the head once lifted, toward where it leads
CORD_LIFT = TOP + 0.025
CORD_ANCHOR = np.array([0.20, 0.42, TOP - 0.10])  # the cable goes over the desk's left edge to its loop round a leg

# -- scoring ---------------------------------------------------------------------------------------------------------
FLUSH = 0.001  # head face within 1 mm of the slot's face
SQUARE_DEG = 8.0  # lock axis within 8 degrees of the slot normal
THROUGH = WALL + 0.0001  # crossbar's back edge just past the wall's back: the T-bar is through the slot
TURNED_DEG = 75.0  # T-bar turned 75 degrees or more: crosswise behind the wall (locked)
MOVED = 0.03  # monitor moved more than 3 cm ...
TILTED_DEG = 10.0  # ... or tipped more than 10 degrees, at any time: 0
ON_DESK_Z = TOP - 0.05  # a monitor below this has left the desk
CLEAR = 0.04  # every gripper tool point this far from the head's centre: let go

# -- start: the lock lying on the desk in front of the monitor, key in, unlocked (as it is left after unlocking) -----
START_XY = (0.24, -0.02)


def rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def start_pose(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Lock origin and rotation at the start for a seed: lying on its barrel, T-bar toward the monitor, key bow flat,
    up to 2 cm off START_XY each way, yawed -20 to 20 degrees (whole mm, whole degrees)."""
    rng = np.random.default_rng(seed)
    dx, dy = (round(v * 1000) / 1000 for v in rng.uniform(-0.02, 0.02, 2))
    yaw = float(round(rng.uniform(-20.0, 20.0)))
    return np.array([START_XY[0] + dx, START_XY[1] + dy, TOP + BARREL_D / 2 + 0.0008]), rot_z(yaw)


def locked_pose() -> tuple[np.ndarray, np.ndarray]:
    """The head seated: face flat on the monitor's back, T-bar through the slot."""
    return np.array([BACK_X, SLOT_Y, SLOT_Z]), np.eye(3)


def seat_of(pos, rot, mon_pos, mon_rot, mon_pos0) -> tuple[float, float, float]:
    """(how far the head's face is past the slot's face along the slot normal (+: in), the T-bar's offset from the
    slot centre in the slot plane, angle in degrees of the lock axis off the slot normal), in the monitor's frame."""
    R0 = np.asarray(mon_rot, float)
    p = R0.T @ (np.asarray(pos, float) - np.asarray(mon_pos, float)) + np.asarray(mon_pos0, float)
    ax = R0.T @ np.asarray(rot, float)[:, 0]
    depth = float(p[0] - BACK_X)
    off = float(math.hypot(p[1] - SLOT_Y, p[2] - SLOT_Z))
    tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(ax[0])))))
    return depth, off, tilt


def cord_force(pos) -> np.ndarray:
    """The cable's pull on the head (N, world): CORD_PULL toward where it leads, once the head is off the desk."""
    if pos[2] < CORD_LIFT:
        return np.zeros(3)
    d = CORD_ANCHOR - np.asarray(pos, float)
    return CORD_PULL * d / np.linalg.norm(d)


def detent_target(angle: float) -> float:
    """The key's detent pulls it to whichever end (0 or 90 degrees) it is nearer."""
    return LOCKED_ANGLE if angle > LOCKED_ANGLE / 2 else 0.0


def through(depth: float, off: float, tilt: float) -> bool:
    """The T-bar's crossbar is behind the wall (through the slot)."""
    return depth + BAR_X0 > THROUGH and off < 0.002 and tilt < SQUARE_DEG


def monitor_moved(pos, rot, pos0) -> bool:
    """The monitor slid more than 3 cm, tipped more than 10 degrees, or left the desk."""
    shift = float(np.linalg.norm((np.asarray(pos, float) - np.asarray(pos0, float))[:2]))
    tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(np.asarray(rot, float)[2, 2])))))
    return shift > MOVED or tilt > TILTED_DEG or pos[2] < ON_DESK_Z


# -- score bookkeeping -----------------------------------------------------------------------------------------------


@dataclass
class Tracker:
    """``step`` once per control step; ``score`` with the end state.

    0 for the rest of the trial once the monitor slides more than 3 cm, tips more than 10 degrees or leaves the desk.
    Otherwise, at the end: 1.0 locked (head flush, T-bar through the slot and turned), the key out of the lock and
    both hands 4 cm or more from the head; 0.8 the same but a hand still at the head; 0.6 locked with the key in it; 0.3 head flush with the T-bar through the slot but not turned (it would come
    straight out); 0.1 the T-bar went through the slot at some point but is not there at the end; else 0."""

    moved: bool = False
    entered: bool = False

    def reset(self) -> None:
        self.moved = False
        self.entered = False

    def step(self, depth: float, off: float, tilt: float, moved: bool) -> None:
        self.moved = self.moved or moved
        self.entered = self.entered or through(depth, off, tilt)

    def score(self, depth: float, off: float, tilt: float, angle_deg: float, key_out: bool, clear: bool = True) -> float:
        if self.moved:
            return 0.0
        seated = through(depth, off, tilt) and depth > -FLUSH
        if seated and angle_deg >= TURNED_DEG:
            if not key_out:
                return 0.6
            return 1.0 if clear else 0.8
        if seated:
            return 0.3
        return 0.1 if self.entered else 0.0
