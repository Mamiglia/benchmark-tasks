"""shiftmodel: the H-pattern gate, its springs, the gear readout and the score. Pure Python + NumPy.

Coordinates of the knob relative to neutral, measured as arc length at the knob centre (m):
``u`` fore-aft (+ away from the robot, toward the "dashboard"), ``v`` sideways (+ toward the robot's left).
The lever has three joints: ``select`` (about the fore-aft axis: a = -v / L), ``shift`` (about the lever's
sideways axis: b = u / L) and ``press`` (the knob pushed down along the lever, 0 to -6 mm).

Gate (Logitech Driving Force Shifter layout, 6 + R):

        1   3   5          u = +U
        |   |   |
        +---+---+          neutral channel, |u| <= C
        |   |   |
        2   4   6/R        u = -U
     v=+P   0   -P

Reverse shares the 6 slot: the knob pushed down 5 mm or more as the lever enters that slot from neutral selects R;
entered without it, the box reads 6. A push made once the lever is already in the slot changes nothing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# -- geometry (m) --------------------------------------------------------------------------------------------------
L = 0.125  # pivot to knob centre
PIVOT_Z = 0.062  # pivot above the shifter's underside
U = 0.030  # fore-aft throw at the knob, neutral to the end of a slot
P = 0.022  # plane spacing at the knob
C = 0.004  # half-width of the slots and of the neutral channel
ENGAGE = 0.9  # a gear reads engaged at 90 % of its slot (27 mm from neutral)
SLOT_IN = 0.5  # the reverse decision is taken as the lever passes half-way into the 6/R slot
PRESS_MAX = 0.006  # knob push-down travel
PRESS_R = 0.005  # push-down that selects reverse
PLANES = {"left": P, "middle": 0.0, "right": -P}
GEARS = {("left", 1): "1", ("left", -1): "2", ("middle", 1): "3", ("middle", -1): "4", ("right", 1): "5",
         ("right", -1): "6"}

# -- springs (N, at the knob) ---------------------------------------------------------------------------------------
# Estimates for a G-series shifter; measure the real unit with a luggage scale at the knob (README) and set here.
SIDE_F0, SIDE_K = 2.0, 100.0  # centring spring toward the 3-4 plane, from the 1-2 side: 2 N + 100 N/m (4.2 N at P)
SIDE_F0_R, SIDE_K_R = 3.5, 150.0  # from the 5-6/R side (stiffer, as on most boxes): 6.8 N at P
DETENT_F = 6.0  # fore-aft detent: peak hold toward neutral, then snaps the lever into the gear
PRESS_F0, PRESS_K = 6.0, 800.0  # push-down spring: 6 N preload + 800 N/m (10 N at 5 mm)


def uv(a: float, b: float) -> tuple[float, float]:
    """(u, v) from the select and shift angles."""
    return L * b, -L * a


def plane_of(v: float) -> str | None:
    """The plane whose slot the knob is lined up with (within C), or None between planes."""
    for name, vk in PLANES.items():
        if abs(v - vk) <= C:
            return name
    return None


def nearest_plane(v: float) -> str:
    return min(PLANES, key=lambda n: abs(v - PLANES[n]))


def gate_limits(u: float, v: float) -> tuple[tuple[float, float], tuple[float, float]]:
    """Joint limits (rad) that make the gate for the knob at (u, v): (select a, shift b).

    In the neutral channel the lever moves sideways over the whole gate; it can go fore-aft only where it is lined
    up with a slot. Once in a slot it can only move along it."""
    a_full = ((-(P + C)) / L, (P + C) / L)
    if abs(u) > C:  # in a slot: sideways limited to that slot
        vk = PLANES[nearest_plane(v)]
        return ((-(vk + C)) / L, (-(vk - C)) / L), (-U / L, U / L)
    if plane_of(v) is not None:  # at a crossing: free both ways
        return a_full, (-U / L, U / L)
    return a_full, (-C / L, C / L)  # between slots: neutral channel only


def side_force(v: float) -> float:
    """Centring spring toward the 3-4 plane (N along +v)."""
    s = math.tanh(v / 0.001)
    if v >= 0:
        return -s * (SIDE_F0 + SIDE_K * abs(v))
    return -s * (SIDE_F0_R + SIDE_K_R * abs(v))


def detent_force(u: float) -> float:
    """Fore-aft detent (N along +u): toward neutral in the first half of a slot, into the gear in the second."""
    half = SLOT_IN * U
    if abs(u) < half:
        return -DETENT_F * u / half
    return DETENT_F * math.copysign(1.0, u)


def press_force(press: float) -> float:
    """Return spring on the knob (N along +press, i.e. up); press is 0 (up) to -PRESS_MAX."""
    return PRESS_F0 + PRESS_K * max(0.0, -press)


def efforts(a: float, b: float, press: float) -> tuple[float, float, float]:
    """Joint efforts: select (N m), shift (N m), press (N)."""
    u, v = uv(a, b)
    return -L * side_force(v), L * detent_force(u), press_force(press)


@dataclass
class Gearbox:
    """What the shifter reports: one of "1".."6", "R" or None (neutral / between)."""

    in_six_slot: str | None = None  # "6" or "R" once the lever is past SLOT_IN in the right-back slot

    def read(self, u: float, v: float, press: float) -> str | None:
        plane = nearest_plane(v) if abs(u) > C else plane_of(v)  # in a slot the gate holds it in that plane
        if plane == "right" and u < -SLOT_IN * U:
            if self.in_six_slot is None:
                self.in_six_slot = "R" if -press >= PRESS_R else "6"
        else:
            self.in_six_slot = None
        if plane is None or abs(u) < ENGAGE * U:
            return None
        g = GEARS[(plane, 1 if u > 0 else -1)]
        return self.in_six_slot if g == "6" and self.in_six_slot else g


ORDER = ("1", "2", "3", "R")
HOLD_S = 1.0  # a gear in the sequence counts once held engaged this long
WRONG_S = 0.25  # any other gear engaged this long ends the sequence


@dataclass
class Sequence:
    """Latched over the whole trial. ``done`` counts the gears of ORDER reached in order, each held HOLD_S. A wrong
    gear (not the next one, not the one just counted) engaged for WRONG_S or more stops all further credit."""

    done: int = 0
    wrong: str | None = None
    gear: str | None = None
    held: float = 0.0
    log: list = field(default_factory=list)

    def feed(self, gear: str | None, dt: float, t: float = 0.0) -> None:
        if gear != self.gear:
            self.gear, self.held = gear, 0.0
        else:
            self.held += dt
        if gear is None or self.wrong is not None:
            return
        nxt = ORDER[self.done] if self.done < len(ORDER) else None
        last = ORDER[self.done - 1] if self.done else None
        if gear == nxt and self.held >= HOLD_S:
            self.done += 1
            self.log.append((round(t, 2), gear))
        elif gear not in (nxt, last) and self.held >= WRONG_S:
            self.wrong = gear
            self.log.append((round(t, 2), f"wrong {gear}"))

    def score(self, final_gear: str | None, foul: bool = False) -> float:
        """0.25 each for 1, 2 and 3 reached in order; 0.25 more for R reached after them and still engaged at the
        end. 0 after a foul."""
        if foul:
            return 0.0
        return 0.25 * min(self.done, 3) + (0.25 if self.done == 4 and final_gear == "R" else 0.0)


def knob_offset(a: float, b: float, press: float = 0.0):
    """Knob centre relative to the pivot (shifter frame, m) and the lever's rotation matrix."""
    import numpy as np

    ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
    rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    r = rx @ ry
    return r @ np.array([0.0, 0.0, L + press]), r


def angles_for(u: float, v: float) -> tuple[float, float]:
    """(a, b) for a knob at (u, v)."""
    return -v / L, u / L
