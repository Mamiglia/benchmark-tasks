"""Unit tests for the pouch geometry and the scorer (no Isaac Sim needed): pytest test_sterile_peel_pouch.py"""

from __future__ import annotations

import pathlib
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import pouchkit as K  # noqa: E402
import task  # noqa: E402

I3 = np.eye(3)


def start_poses():
    tr = K.start_transforms()
    return {b: (tr[i, :3], K.quat_to_matrix(tr[i, 3:])) for i, b in enumerate(K.BODIES)}


def test_seal_anchors_coincide_at_start():
    P = start_poses()
    for b in range(K.N + 1):
        for pb, pl, fb, fl in K.seal_anchors(b):
            a = P[pb][0] + P[pb][1] @ np.asarray(pl)
            c = P[fb][0] + P[fb][1] @ np.asarray(fl)
            assert np.allclose(a, c, atol=1e-9)
    assert not any(K.seals_parted(P)) and K.peeled_mm(K.seals_parted(P)) == 0.0


def test_peel_progress_counts_from_the_chevron():
    assert K.peeled_mm([True] + [False] * K.N) == 0.0
    assert K.peeled_mm([True, True, True] + [False] * (K.N - 2)) == 2 * K.STRIP_L * 1000
    assert K.peeled_mm([True] * (K.N + 1)) == K.N * K.STRIP_L * 1000
    assert K.peeled_mm([False, True, True] + [False] * (K.N - 2)) == 0.0


def test_pouch_lies_flat_on_rest_and_clears_drape_at_start():
    P = start_poses()
    assert np.allclose(K.quat_to_matrix(K.FLAT_Q), K.FLAT_R)
    strips = np.concatenate([K.box_corners(*P[f"{p}_{k}"], (0, 0, 0), K.STRIP_SIZE) for p in K.PLIES
                             for k in range(K.N)])
    assert abs(strips[:, 2].min() - K.REST_TOP) < 1e-9  # film's outer face on the rest
    assert np.ptp(strips[:, 2]) < K.GAP + 2 * K.PLY_T + 1e-9  # flat: two plies and the pocket
    rest_y0 = K.REST_C[1] - K.REST_SIZE[1] / 2
    assert strips[:, 1].min() >= K.CHEVRON_Y - 1e-9 and strips[:, 1].max() < rest_y0 + K.REST_SIZE[1]
    film_tab = K.box_corners(*P["film_tab"], (0, 0, 0), K.TAB_SIZE)
    paper_tab = K.box_corners(*P["paper_tab"], (0, 0, 0), K.TAB_SIZE)
    assert film_tab[:, 1].max() < rest_y0 and film_tab[:, 2].min() > K.TOP  # hangs over the edge, clear of the table
    assert paper_tab[:, 2].min() >= strips[:, 2].max() - 1e-9  # turned up
    assert not K.pouch_touches_drape(K.pouch_corners(P))
    f = K.forceps_corners(*P["forceps"])
    assert strips[:, 2].min() + K.PLY_T < f[:, 2].min() and f[:, 2].max() < strips[:, 2].max() - K.PLY_T


def test_forceps_inside_pocket():  # upright design frame
    fc = K.forceps_center()
    pts = K.forceps_corners(fc, I3)
    assert np.abs(pts[:, 1]).max() < K.GAP / 2  # between the plies' inner faces
    assert pts[:, 2].min() > K.POUCH_BOTTOM + K.SEAM_W  # on the bottom seam
    assert pts[:, 2].max() < K.Z_SEAL_TOP
    blade = fc + (K.BLADE_DX, 0, 0)
    assert K.gap_to_forceps(blade, fc, I3) == 0.0
    assert abs(K.gap_to_forceps(blade + (0, 0.05, 0), fc, I3) - (0.05 - K.BLADE_T / 2)) < 1e-9


def test_drape_border():
    c = np.array([*K.DRAPE_C, K.DRAPE_TOP])
    assert K.over_drape(c, K.BORDER)
    edge = c + (K.DRAPE_HALF[0] - 0.01, 0, 0)
    assert K.over_drape(edge) and not K.over_drape(edge, K.BORDER)
    assert not K.over_drape(c + (K.DRAPE_HALF[0] + 0.001, 0, 0))
    assert K.pads_touch_drape({"l": np.array([c + (0, 0, 0.005)])})
    assert not K.pads_touch_drape({"l": np.array([c + (0, 0, 0.05)]), "r": np.array([(0.1, 0.0, c[2])])})


def state(at=(0.52, 0.0), z=None, parted=None, peeled=0.0, pouch=False, forceps=False, drape=False, v=0.0):
    z = K.DRAPE_TOP + K.BLADE_T / 2 if z is None else z
    R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])  # lying flat on its side
    pos = np.array([at[0], at[1], z])
    parted = tuple([True] * (K.N + 1)) if parted is None else parted
    return {"pouch_rig": SimpleNamespace(forceps_pts=K.forceps_corners(pos, R), lin_vel=np.array([v, 0, 0]),
                                         parted=parted, peeled_mm=peeled, pouch_on_drape=pouch,
                                         touched_forceps=forceps, touched_drape=drape)}


def test_score_outcomes():
    assert task.score(state()) == 0.75 and task.success(state())
    assert task.score(state(at=(0.52 + K.DRAPE_HALF[0] - 0.02, 0.0))) == 0.5  # on the non-sterile border
    assert task.score(state(pouch=True)) == 0.25
    assert task.score(state(forceps=True)) == 0.0
    assert task.score(state(drape=True)) == 0.0
    assert task.score(state(v=0.1)) == 0.2  # still moving: not yet landed
    assert task.score(state(z=K.DRAPE_TOP + 0.02)) == 0.2  # on the pouch / in the air, seal all open
    closed = tuple([True] * 7 + [False] * (K.N - 6))
    assert task.score(state(z=0.9, parted=closed, peeled=6 * K.STRIP_L * 1000)) == 0.1
    assert task.score(state(z=0.9, parted=tuple([False] * (K.N + 1)))) == 0.0
    assert task.score(state(at=(0.0, 0.0))) == 0.2  # off the drape, seal open
