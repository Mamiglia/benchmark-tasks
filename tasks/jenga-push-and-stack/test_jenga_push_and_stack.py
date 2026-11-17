"""Unit tests for the scorer and the tower layout. No Isaac Sim needed: pytest test_jenga_push_and_stack.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task  # noqa: E402
from jengakit import BlockState  # noqa: E402

M = "b10_1"  # a middle block of level 10 (runs along y)


def quat_z(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def state(spec, pos=None, yaw=0.0, left=False, right=False, vel=0.0):
    pos = np.array(spec.pos if pos is None else pos, float)
    sx, sy, sz = spec.size
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    half = np.array([c * sx + s * sy, s * sx + c * sy, sz]) / 2
    return BlockState(pos, quat_z(yaw), np.array([vel, 0, 0]), np.zeros(3), pos - half, pos + half, left, right)


def start():
    return {b.name: state(b) for b in task.BLOCKS}


SPEC = {b.name: b for b in task.BLOCKS}


def run(frames):
    task._reset()
    s = None
    for f in frames:
        s = task.score(f)
    return s


def moved(objs, n, **kw):
    out = dict(objs)
    out[n] = state(SPEC[n], **kw)
    return out


def on_top_pos(dx=-0.018, dy=0.0):
    b = SPEC[M]
    return (task.TOWER_XY[0] + dx, task.TOWER_XY[1] + dy, task.TOWER_TOP + b.size[2] / 2 + 0.0005)


def test_layout():
    assert len(task.BLOCKS) == 54 and len(set(task.NAMES)) == 54
    h = task.thicknesses(0)
    assert np.all(np.abs(h - task.HEIGHT) <= task.TOL)
    assert np.allclose(h, task.thicknesses(0)) and not np.allclose(h, task.thicknesses(1))
    assert 0.268 < task.TOWER_TOP - task.TOP < 0.276
    assert SPEC["b01_1"].size[0] == task.LENGTH and SPEC["b18_1"].size[1] == task.LENGTH


def test_untouched_scores_zero():
    assert run([start()] * 3) == 0.0


@pytest.mark.parametrize("dy,expect", [(-0.003, 0.0), (-0.008, 0.1), (-0.040, 0.3)])
def test_pushed(dy, expect):
    b = SPEC[M]
    assert run([start(), moved(start(), M, pos=(b.pos[0], b.pos[1] + dy, b.pos[2]))]) == expect


def test_out_on_table_is_06():
    b = SPEC[M]
    path = [moved(start(), M, pos=(b.pos[0], b.pos[1] - d, b.pos[2])) for d in (0.01, 0.04, 0.08)]
    table = moved(start(), M, pos=(0.30, -0.20, task.TOP + 0.0075))
    assert run([start(), *path, table]) == 0.6


def test_on_top_crosswise_is_full():
    final = moved(start(), M, pos=on_top_pos(), yaw=math.pi / 2)  # a y block turned to run along x
    assert run([start(), final]) == 1.0


def test_on_top_but_held_or_parallel_or_off_centre():
    assert run([start(), moved(start(), M, pos=on_top_pos(), yaw=math.pi / 2, right=True)]) == 0.6
    assert run([start(), moved(start(), M, pos=on_top_pos(), yaw=0.0)]) == 0.6  # along the top level
    assert run([start(), moved(start(), M, pos=on_top_pos(dx=-0.035), yaw=math.pi / 2)]) == 0.6
    assert run([start(), moved(start(), M, pos=on_top_pos(), yaw=math.pi / 2, vel=0.05)]) == 0.6


def test_other_block_disturbed_caps():
    final = moved(start(), M, pos=on_top_pos(), yaw=math.pi / 2)
    o = SPEC["b11_0"]
    nudged = moved(final, "b11_0", pos=(o.pos[0] + 0.006, o.pos[1], o.pos[2]))
    assert run([start(), nudged]) == 0.1
    shoved = moved(final, "b11_0", pos=(o.pos[0] + 0.012, o.pos[1], o.pos[2]))
    assert run([start(), shoved, final]) == 0.1  # put back: still latched


def test_tower_fell_is_zero():
    final = moved(start(), M, pos=on_top_pos(), yaw=math.pi / 2)
    o = SPEC["b15_2"]
    fell = moved(final, "b15_2", pos=(o.pos[0], o.pos[1] + 0.05, o.pos[2] - 0.05))
    assert run([start(), fell, final]) == 0.0


def test_two_hands_on_the_tower_is_zero():
    final = moved(start(), M, pos=on_top_pos(), yaw=math.pi / 2)
    b = SPEC[M]
    pushing = moved(start(), M, pos=(b.pos[0], b.pos[1] - 0.01, b.pos[2]), left=True)
    braced = moved(pushing, "b13_1", right=True)
    assert run([start(), braced, final]) == 0.0
    # one hand after the other is fine; so is holding the taken block once it is out of the tower
    out = moved(start(), M, pos=(0.30, -0.15, b.pos[2]), right=True)
    out = moved(out, "b18_1", left=True)
    assert run([start(), pushing, out, final]) == 1.0


def test_top_level_block_is_not_takeable():
    t = SPEC["b18_1"]
    final = moved(start(), "b18_1", pos=(t.pos[0], t.pos[1], t.pos[2] + 0.016), yaw=math.pi / 2)
    assert run([start(), final]) == 0.0
