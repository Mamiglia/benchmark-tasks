"""On the desk in front of the arms: an ivory rotary-dial telephone (GPO 746 style, 21.5 x 16 x 14.5 cm, 1 kg)
standing loose, its front with the dial facing the arms, a little right of centre. The dial face slopes 35 degrees;
the finger wheel (9.6 cm across) has ten 17 mm finger holes, 1 near the top right through 0 at the bottom, and a
chrome finger stop at about half past four. The handset lies off its cradle on the desk to the left of the phone.
To the right, an A6 notepad with a yellow HB pencil (190 mm, 7 mm across) lying on it, its eraser end toward the
arms.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import dialmodel as M  # noqa: E402
from dialkit import Handset, Pencil, Phone  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Dial 4, 1, 1 on the rotary phone with the pencil. For each digit, put the pencil's eraser end in "
               "the digit's hole, pull the dial round to the finger stop, then take the pencil out and let the dial "
               "run back on its own before the next digit.")

PHONE_XY = (0.40, -0.03)
PAD_XY, PAD_SIZE = (0.25, -0.245), (0.105, 0.148, 0.008)
PENCIL_LEN, PENCIL_W, PENCIL_MASS = 0.190, 0.007, 0.006

PHONE = Phone("phone", pos=(*PHONE_XY, TOP), size=M.BODY, color=(0.88, 0.84, 0.72))
HANDSET = Handset("handset", pos=(0.40, 0.17, TOP + 0.02), size=(0.23, 0.06, 0.04), color=(0.88, 0.84, 0.72))
NOTEPAD = Box("notepad", pos=(*PAD_XY, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=(0.96, 0.96, 0.90))
PENCIL = Pencil("pencil", pos=(PAD_XY[0], PAD_XY[1] + 0.01, TOP + PAD_SIZE[2] + PENCIL_W / 2 + 0.0005),
                size=(PENCIL_LEN, PENCIL_W, PENCIL_W), mass=PENCIL_MASS, color=(0.95, 0.75, 0.10))
OBJECTS = [PHONE, HANDSET, NOTEPAD, PENCIL]


def score(objects) -> float:
    """0 after a foul (latched for the whole trial: the phone slid 3 cm, tipped 10 degrees or lifted 1 cm; a gripper
    on the dial: within 15 mm of its face, inside 5.2 cm of its centre). Otherwise the number of leading digits
    dialed correctly, / 3. A digit is the number of pulses a pull sends: one for each hole the pull brought to
    within 15 degrees (about 1 cm) of the finger stop. A pull held back or pushed on again on its run-back dials a
    wrong digit. A pull that sends no pulse dials nothing. A fourth digit makes the number wrong (at most 2/3)."""
    st = objects["phone"]
    return M.score_value(list(st.digits), bool(st.fouls))


def success(objects) -> bool:
    """4, 1, 1 dialed and the dial back at rest."""
    return score(objects) == 1.0 and objects["phone"].theta_deg < M.REST


def controller(bases, dt):
    """yam_scripted plays this closed-loop controller (dialdemo.py)."""
    import dialdemo

    return dialdemo.Controller(bases, dt)
