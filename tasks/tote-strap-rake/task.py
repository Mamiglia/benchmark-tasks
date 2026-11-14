"""On the station table, in front of the arms, stands a steel shelf: a deck 30 cm deep (x 36 to 66 cm) and 76 cm wide
(y -32 to +44 cm), 15 cm above the table, on four posts. On the deck stands a blue polypropylene storage tote,
26 x 18 x 10 cm, its long side facing the arms, with a hand-hold ledge along the top of each end wall. An orange
webbing strap is sewn to the tote's back wall at its left corner and ends in a steel ring. Left of the tote, a
steel hand rake lies on its back on the deck, handle to the left, its three tines pointing up; the strap's ring
has dropped over the middle tine. The tote, strap and rake shift together by up to 2 cm (x) and 3 cm (y) with the
trial seed.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

# Idea first proposed in robocurve/benchmark-tasks#295; independent implementation.

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import totekit as K  # noqa: E402
from yam_arena import station  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = K.TOP
INSTRUCTION = "Take the tote down from the shelf and put it on the table."

RIG = K.ToteRig("tote_rig", pos=(0.51, 0.06, K.DECK_TOP + 0.05), size=(0.30, 0.76, 0.10))
OBJECTS = [RIG]

# -- scoring ---------------------------------------------------------------------------------------------------------
# The table top's footprint (station table) and the part of it in front of the shelf.
TABLE_X = (station.TABLE_CENTER[0] - station.TABLE_SIZE[0] / 2, station.TABLE_CENTER[0] + station.TABLE_SIZE[0] / 2)
TABLE_Y = (station.TABLE_CENTER[1] - station.TABLE_SIZE[1] / 2, station.TABLE_CENTER[1] + station.TABLE_SIZE[1] / 2)
ON_TABLE_M = 0.01  # every bottom corner of the tote within 1 cm above the table top
UPRIGHT = 0.996  # the tote's up axis within 5 degrees of vertical (cos 5 deg)
CLEAR_M = 0.03  # every finger pad and tool point at least 3 cm from the tote (ledges included)
OPEN_FRAC = 0.8  # both grippers at least 80% open
STILL_V, STILL_W = 0.01, 0.05  # m/s, rad/s
RAKE_REST_M = 0.01  # the rake's lowest point within 1 cm of the deck top, and still


def tote_on_table(s) -> bool:
    """The tote stands upright on the table top in front of the shelf: all four bottom corners within 1 cm above the
    table top, inside the table's footprint and in front of the deck's front edge; up axis within 5 degrees."""
    b = s.corners[:4]
    return bool(s.tote_R[2, 2] >= UPRIGHT and (b[:, 2] >= TOP - 0.003).all() and (b[:, 2] <= TOP + ON_TABLE_M).all()
                and (b[:, 0] >= TABLE_X[0]).all() and (b[:, 0] <= K.DECK_X[0]).all()
                and (b[:, 1] >= TABLE_Y[0]).all() and (b[:, 1] <= TABLE_Y[1]).all())


def released(s) -> bool:
    """Both grippers at least 80% open and every finger pad and tool point 3 cm or more from the tote."""
    gap = min(K.gap_to_tote(p, s.pos, s.tote_R) for pads in s.pads.values() for p in pads)
    return min(s.open_frac.values()) >= OPEN_FRAC and gap >= CLEAR_M


def still(s) -> bool:
    return float(np.linalg.norm(s.lin_vel)) < STILL_V and float(np.linalg.norm(s.ang_vel)) < STILL_W


def rake_on_deck(s) -> bool:
    """The rake rests on the deck: lowest point within 1 cm of the deck top, no point past an edge, still."""
    return (not K.rake_past_edge(s.rake_pts) and float(s.rake_pts[:, 2].min()) <= K.DECK_TOP + RAKE_REST_M
            and s.rake_vel < STILL_V)


def score(objects) -> float:
    """Latched over the whole trial, 0 if: the rake went off the shelf (a point of it more than 1 cm past any edge of
    the deck, front, back or sides, or more than 1 cm below the deck top); the rake was dragged (moved 3 cm or more from
    its start while the ring was still round a tine); the tote fell (faster than 0.6 m/s down) or went below the table
    top. Otherwise, with the ring off every tine and the rake resting on the deck: 1.0 (success) when the tote stands
    upright on the table in front of the shelf, released and still, held 1 s; 0.5 on the table but held or moving;
    0.25 not on the table. With the ring still round a tine: 0."""
    s = objects["tote_rig"]
    if s.rake_off or s.dragged or s.tote_dropped or s.tote_floor:
        return 0.0
    if s.ring_caught or not rake_on_deck(s):
        return 0.0
    if tote_on_table(s):
        return 0.75 if released(s) and still(s) else 0.5
    return 0.25


def success(objects) -> bool:
    """On the table, released and still, the ring free, the rake on the deck; must hold 1 s (SUCCESS_HOLD_S)."""
    return score(objects) == 0.75


# -- scripted demo (closed loop; see totedemo.py) --------------------------------------------------------------------


def controller(bases, dt):
    """yam_scripted plays this: it sees the top camera image and the joint positions only."""
    import totedemo

    return totedemo.Controller(bases, dt)
