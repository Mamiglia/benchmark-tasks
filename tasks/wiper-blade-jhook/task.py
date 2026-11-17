"""On the station table: a windshield panel (clear acrylic, tinted grey-blue here), sloping up away from the arms at 27
degrees on a stand. A black J-hook wiper arm lies across its lower part, pivot on the right, with a worn 18-inch
(45 cm) conventional wiper blade (dull grey frame) hooked on its end. The arm is spring-loaded against the glass and
does not stay up by itself. To the left of the windshield, a new blade of the same model (black frame, its J-hook
adapter fitted) stands on its rubber in the slot of a cardboard parts tray on the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from hookmodel import (ALPHA, ARM_LEN, GLASS_T, LOW_DEG, PIVOT_Z, R_G, RZ90, SEAT0, SLIDE_LO, TAB_PRESSED_M,  # noqa: E402
                       ArmMonitor, HookMonitor, LatchParams, Layout, CLICK_M)
from wiperkit import Link, Slide, matrix_to_quat, on_physics, on_reset, on_step, part, robot_hands  # noqa: E402
from wiperkit import set_link, set_link_drive  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Replace the wiper blade with the new one and lay the arm back on the windshield."

GLASS_C, BLACK, STAND = (0.62, 0.72, 0.78), (0.07, 0.07, 0.08), (0.32, 0.33, 0.35)
STEEL, OLD_FRAME, OLD_RUBBER = (0.55, 0.56, 0.58), (0.50, 0.47, 0.42), (0.20, 0.19, 0.18)
NEW_FRAME, ADAPTER_NEW, ADAPTER_OLD, TAB_C = (0.10, 0.10, 0.11), (0.25, 0.25, 0.27), (0.30, 0.29, 0.27), (0.85, 0.85, 0.80)

# -- the windshield (glass frame G: x up the slope, y along the lower edge, z out of the glass) -----------------------
G_ORIGIN = (0.20, -0.02, TOP + 0.030)  # world point: middle of the glass's lower edge, on its top surface
GLASS_U, GLASS_W = 0.36, 0.68
ARM_U = 0.11  # the arm lies 11 cm up the glass from its lower edge
PIVOT = (ARM_U, -0.30, PIVOT_Z)
LAYOUT = Layout(G_ORIGIN, PIVOT)

ARM_BOXES = (  # arm frame: origin on the hinge axis, y along the arm
    ((0.0, 0.003, 0.0), (0.026, 0.05, 0.034), BLACK),  # pivot head
    ((0.0, 0.155, 0.0), (0.012, 0.26, 0.010), BLACK),  # shaft (U-channel), 44-54 mm above the glass
    ((0.0, 0.295, 0.0), (0.012, 0.02, 0.010), BLACK, False),  # its last 2 cm (drawn only)
    ((0.0, 0.318, -0.009), (0.010, 0.026, 0.008), BLACK, False),  # drop to the hook
    ((0.0, 0.343, -0.0135), (0.009, 0.032, 0.0025), STEEL, False),  # hook strip, on the adapter's top
    ((0.0, 0.36125, -0.0185), (0.009, 0.0025, 0.0125), STEEL, False),  # the bend, round the adapter's nose
    ((0.0, 0.3515, -0.0237), (0.009, 0.022, 0.0025), STEEL, False),  # return leg, in the adapter's slot
)
ARM = Slide(name="arm", origin=PIVOT, axis=(1.0, 0.0, 0.0), boxes=ARM_BOXES, lower=math.radians(LOW_DEG),
            upper=math.radians(45.0), stiffness=1.2, damping=0.01, mass=0.30, revolute=True,
            target=-1.73, armature=0.002)
WINDSHIELD = part(
    "windshield", G_ORIGIN,
    (((GLASS_U / 2, 0.0, -GLASS_T / 2), (GLASS_U, GLASS_W, GLASS_T), GLASS_C),
     ((PIVOT[0], PIVOT[1], 0.012), (0.03, 0.03, 0.024), BLACK)),  # pivot post
    mass=3.0, quat=matrix_to_quat(R_G), friction=0.8, slides=(ARM,), fixed=True)

# -- stand: fixed blocks under the glass's lower and upper edges ------------------------------------------------------
_lo_x, _hi = 0.20, np.asarray(G_ORIGIN) + R_G @ np.array([GLASS_U, 0.0, -GLASS_T])
STANDS = [
    Box("cowl", pos=(0.185, -0.02, TOP + 0.013), size=(0.04, GLASS_W, 0.026), color=BLACK),
    Box("stand_rear", pos=(_hi[0] - 0.012, -0.02, (TOP + _hi[2]) / 2), size=(0.03, GLASS_W, _hi[2] - TOP),
        color=STAND),
]

# -- blades (blade frame: origin at the rubber's wiping edge, middle; x along the blade, z up through the adapter) ----
BLADE_L = 0.45
TAB = (-0.010, 0.020, 0.0315)  # release tab centre, blade frame: a lever off the adapter's side (robot side)


def blade_boxes(frame, rubber, adapter):
    return (
        ((0.0, 0.0, 0.004), (BLADE_L, 0.006, 0.008), rubber),  # rubber squeegee
        ((0.0, 0.0, 0.012), (BLADE_L - 0.01, 0.010, 0.008), frame),  # spine
        ((0.0, 0.0, 0.020), (0.20, 0.010, 0.008), frame),  # centre bridge
        ((-0.003, 0.0, 0.029), (0.030, 0.011, 0.010), adapter),  # J-hook adapter, nose at x +12 mm
        ((-0.010, 0.010, 0.0315), (0.010, 0.004, 0.006), adapter),  # tab's root
    )


def tab_slide():
    return Slide(name="tab", origin=TAB, axis=(0.0, 0.0, 1.0), boxes=(((0.0, 0.004, 0.0), (0.010, 0.016, 0.003), TAB_C),),
                 lower=-0.0025, upper=0.0, stiffness=1500.0, damping=5.0, mass=0.002, target=0.002)


LATCH = LatchParams()
SEAT_ROT0 = RZ90  # blade axes in the arm frame


def blade_links(name):
    return (Link(name=f"{name}_lock", to="windshield/arm", pos0=tuple(SEAT0), rot0=tuple(map(tuple, SEAT_ROT0)),
                 enabled=name == "old_blade", break_force=LATCH.break_n),
            Link(name=f"{name}_slide", to="windshield/arm", pos0=tuple(SEAT0), rot0=tuple(map(tuple, SEAT_ROT0)),
                 slide=True, lower=SLIDE_LO, upper=0.0005, damping=LATCH.slide_damping, enabled=False))


_seat_o, _seat_r = LAYOUT.seat(0.0)
OLD = part("old_blade", _seat_o, blade_boxes(OLD_FRAME, OLD_RUBBER, ADAPTER_OLD), mass=0.11,
           quat=matrix_to_quat(_seat_r), friction=0.9, slides=(tab_slide(),), links=blade_links("old_blade"),
           com=(0.0, 0.0, 0.012))
NEW_XY = (0.33, 0.45)
TRAY_WALL, TRAY_IN, TRAY_BASE = 0.012, 0.012, 0.003  # cardboard tray: walls 12 mm tall, 12 mm slot, 3 mm floor
CARD = (0.72, 0.60, 0.42)
TRAY = [
    Box("tray_floor", pos=(NEW_XY[0], NEW_XY[1], TOP + TRAY_BASE / 2), size=(0.48, 0.04, TRAY_BASE), color=CARD),
    *[Box(f"tray_wall_{i}", pos=(NEW_XY[0], NEW_XY[1] + sy * (TRAY_IN / 2 + 0.0015), TOP + TRAY_BASE + TRAY_WALL / 2),
          size=(0.48, 0.003, TRAY_WALL), color=CARD) for i, sy in enumerate((-1, 1))],
]
NEW = part("new_blade", (NEW_XY[0], NEW_XY[1], TOP + TRAY_BASE + 0.0003), blade_boxes(NEW_FRAME, BLACK, ADAPTER_NEW),
           mass=0.11, quat=(0.0, 0.0, 1.0, 0.0), friction=0.9, slides=(tab_slide(),), links=blade_links("new_blade"),
           com=(0.0, 0.0, 0.012))  # running along -x, release tab toward the windshield
BLADES = ("old_blade", "new_blade")
OBJECTS = [WINDSHIELD, *STANDS, *TRAY, OLD, NEW]

# -- run-time mechanics -------------------------------------------------------------------------------------------
MON = {"old_blade": HookMonitor(True, LATCH), "new_blade": HookMonitor(False, LATCH)}
ARM_MON = ArmMonitor()
HOLD_OPEN, HOLD_MIN = 0.014, 0.005  # a gripper closed on the blade (10-11 mm wide): opening between these (m) ...
HOLD_REACH = 0.025  # ... with its tool point this close to the blade's body: holding it
_RT: dict = {}
DT = 1 / 240
MECH_EVERY = 4  # the latch and arm rules run at 60 Hz (reading the state every 240 Hz step is slow)


def _reset(environment=None) -> None:
    for m in MON.values():
        m.reset()
    ARM_MON.reset()
    _RT.update(steps=0, fell="", theta=0.0, states={}, hands={}, held={}, rel={})
    if environment is not None:
        for n in BLADES:
            set_link(f"{n}_lock", MON[n].state == "latched")
            set_link(f"{n}_slide", False)
            set_link_drive(f"{n}_slide", target=-CLICK_M, stiffness=0.0, max_force=LATCH.click_n)


on_reset(_reset)
_reset()


def blade_held(blade, hands) -> str:
    for side, (tcp, _, opening) in hands.items():
        if not HOLD_MIN <= opening <= HOLD_OPEN:  # closed on something, not on nothing
            continue
        p = blade.to_local(tcp)
        if (abs(p[0]) <= BLADE_L / 2 and abs(p[1]) <= HOLD_REACH and -0.005 <= p[2] <= 0.035):
            return side
    return ""


@on_physics
def _mechanics(environment) -> None:
    from yam_arena.scene import object_states

    _RT["steps"] += 1
    if _RT["steps"] % MECH_EVERY:
        return
    s = object_states(environment, ["windshield", *BLADES])
    hands = robot_hands(environment)
    ws = s["windshield"]
    theta, theta_dot = float(ws.slides[0]), float(ws.slide_vel[0])
    _RT.update(theta=theta, theta_dot=theta_dot, states=s, hands=hands)
    _RT["mon"] = {n: MON[n].state for n in BLADES}
    for n in BLADES:
        b = s[n]
        rel, ang = LAYOUT.rel(theta, b.pos, b.rotation())
        held = blade_held(b, hands)
        _RT["rel"][n], _RT["held"][n] = (rel, ang), held
        for cmd, on in MON[n].step(rel, ang, bool(b.slides[0] <= -TAB_PRESSED_M), bool(held)):
            if cmd == "lock":
                set_link(f"{n}_lock", on)
            elif cmd == "slide":
                set_link(f"{n}_slide", on)
            elif cmd == "click":
                set_link_drive(f"{n}_slide", stiffness=LATCH.click_k if on else 0.0)
        if b.pos[2] < TOP - 0.05 and not _RT["fell"]:
            _RT["fell"] = n
    ARM_MON.step(theta, theta_dot, any(m.state != "free" for m in MON.values()))
    if os.environ.get("WIPER_DEBUG") and _RT["steps"] % 24 == 0:
        print(f"[wiper] t {_RT['steps'] * DT:6.2f} arm {math.degrees(theta):6.2f} deg v {theta_dot:6.2f} "
              + " ".join(f"{n[:3]} {MON[n].state} rel {np.round(_RT['rel'][n][0] * 1000, 1)} ang {_RT['rel'][n][1]:.1f} "
                         f"tab {s[n].slides[0] * 1000:.1f} held {_RT['held'][n] or '-'} z {s[n].pos[2] - TOP:.3f}"
                         for n in BLADES)
              + f" peak {ARM_MON.peak_mps:.2f} fail {failure()!r} open "
              + " ".join(f"{k[0]} {v[2] * 1000:.0f} tcp {np.round(s['old_blade'].to_local(v[0]) * 1000, 0)} x {np.round(s['old_blade'].rotation().T @ v[1][:, 0], 2)}" for k, v in hands.items()), flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------
REST_DEG = (-0.6, 1.0)  # arm resting on the glass: hinge angle within this of the latched blade's rest (0)
CLEAR_M = 0.03
TABLE_ZONE_M = 0.015  # a blade on the table: its lowest point this close to the table top
PANEL_X = (0.165, _hi[0] + 0.03)  # windshield footprint (cowl to stand), world x


def failure() -> str:
    if MON["old_blade"].broken or MON["new_blade"].broken:
        return "blade forced off the hook"
    if ARM_MON.failed:
        return ARM_MON.failed
    if _RT.get("fell"):
        return f"{_RT['fell']} off the table"
    return ""


def on_table(b) -> bool:
    return bool(b.low[2] < TOP + TABLE_ZONE_M and not (PANEL_X[0] < b.pos[0] < PANEL_X[1]
                                                       and abs(b.pos[1] + 0.02) < GLASS_W / 2 + 0.01))


def _points():
    """Sample points on the arm and both blades (world), for the hands-clear check."""
    theta = _RT.get("theta", 0.0)
    pts = [LAYOUT.arm_point(theta, (0.0, y, 0.0)) for y in np.linspace(0.0, ARM_LEN, 10)]
    for n in BLADES:
        b = _RT["states"].get(n)
        if b is not None:
            pts += [b.to_world((x, 0.0, 0.015)) for x in np.linspace(-BLADE_L / 2, BLADE_L / 2, 10)]
    return np.array(pts)


def stage(objects) -> float:
    if failure():
        return 0.0
    old, new = objects["old_blade"], objects["new_blade"]
    if not (MON["old_blade"].state == "free" and on_table(old) and not _RT["held"].get("old_blade")):
        return 0.0
    if MON["new_blade"].state != "latched":
        return 0.25
    deg = math.degrees(_RT.get("theta", 0.0))
    if not (REST_DEG[0] <= deg <= REST_DEG[1] and abs(_RT.get("theta_dot", 0.0)) < 0.05):
        return 0.5
    hands = _RT.get("hands", {})
    pts = _points()
    if hands and all(np.min(np.linalg.norm(pts - tcp, axis=1)) >= CLEAR_M for tcp, _, _ in hands.values()):
        return 1.0
    return 0.75


def score(objects) -> float:
    """0 if at any time the arm landed on the glass faster than 0.75 m/s at the hook (about a 3 cm fall on its spring),
    the bare hook touched the glass, a latched blade was forced off the hook, or a blade left the table. Otherwise:
    0.25 the old blade off the hook and lying on the table (not on the windshield), let go; 0.5 and the new blade
    latched on the hook; 0.75 and the arm resting on the glass on the new blade; 1.0 and both grippers 3 cm or more
    from the arm and both blades (success, held 1 s)."""
    return stage(objects)


def success(objects) -> bool:
    return stage(objects) == 1.0


# -- scripted demo ---------------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from wiperdemo import Controller

    return Controller(bases, dt, sim=_RT, layout=LAYOUT, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
