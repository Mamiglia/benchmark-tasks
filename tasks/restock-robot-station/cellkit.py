"""cellkit: the other robot of the cell (the left YAM, a machine loader on its own program) and the cell's monitor,
for a yam_arena task file.

yam_arena gives a policy both arms. This module hooks yam_arena (without modifying it) so that:

- ``YamArenaEmbodiment.step`` replaces the policy's left-arm command with the loader's own program
  (:class:`Loader`): every cycle it reaches into the infeed nest, closes, carries whatever it holds to the machine's
  infeed chute, opens and goes back to its waiting pose. Its cycle times come from the trial seed
  (:func:`schedule`), so they differ between trials and cannot be learned as a fixed clock.
- Before every PHYSICS step (240 Hz, a wrapper on ``InteractiveScene.write_data_to_sim``) PhysX contact reports
  between every link of the right arm and every link of the left arm are read; any contact latches ``touched``.
- ``yam_arena.scene.object_states`` feeds each control step's blank positions and the right gripper's tool point to
  the :class:`Monitor`, which records which supplied blank lay in the nest each time the loader closed on it.
- ``yam_arena.scripted.YamScriptedPolicy`` runs the task module's ``demo_controller`` closed loop.
- ``reset(..., seed=S)`` rebuilds the loader's program for seed S and clears the monitor.

The top half (``schedule``, ``loader_waypoints``, ``Monitor``) is plain NumPy and is what the tests exercise.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import os
import sys
from dataclasses import dataclass, field

import numpy as np

_DEBUG = bool(os.environ.get("CELL_DEBUG"))


# -- the cell ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Cell:
    """Geometry the loader and the monitor need (world frame, m)."""

    table_z: float
    nest_xy: tuple[float, float]
    nest_half: float  # half the pocket's inner width
    chute_inner: tuple[float, float, float, float]  # x0, x1, y0, y1
    chute_wall_h: float
    blank_h: float
    blanks: tuple[str, ...]  # the five the controlled arm must supply
    preload: str  # the blank in the nest at the start; never credited
    home: tuple[float, float, float]  # the loader's waiting pose (tool point)
    first_close: tuple[float, float]  # the first cycle's close time is drawn from this range (s)
    gap: tuple[float, float]  # each later cycle starts this long after the one before (s), drawn per cycle
    cycles: int = 6
    clear_m: float = 0.08  # the right tool point must be this far (xy) from the nest when the loader closes


# The loader's cycle, relative to the moment its fingers have closed in the nest (s).
APPROACH_S = 1.6  # from home to above the nest
DESCEND_S = 1.0
CLOSE_S = 0.6
LIFT_S = 0.8
CARRY_S = 1.6
LOWER_S = 0.5
OPEN_S = 0.4
RISE_S = 0.5
RETURN_S = 1.2
LEAD_S = APPROACH_S + DESCEND_S + CLOSE_S  # leaves home this long before it has closed
GRIP_DZ = 0.030  # tool point above the table when it grips a blank in the nest
OVER_DZ = 0.13  # tool point above the table over the nest
CHUTE_OVER_DZ = 0.16  # ... and over the chute (the held blank clears its 10 cm walls by 3 cm)
DROP_DZ = 0.085  # tool point above the table when it lets go in the chute (blank bottom 4.5 cm below the rim)


def schedule(seed: int, cell: Cell) -> tuple[float, ...]:
    """The loader's close times (s from the start of the trial) for a seed, to 0.1 s."""
    rng = np.random.default_rng([seed, 776])
    t = [round(float(rng.uniform(*cell.first_close)), 1)]
    for _ in range(cell.cycles - 1):
        t.append(round(t[-1] + float(rng.uniform(*cell.gap)), 1))
    return tuple(t)


def loader_waypoints(closes, cell: Cell, start=None):
    """The loader's whole trial as yam_arena.demo Waypoints (it starts from READY)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    nx, ny = cell.nest_xy
    cx, cy = (cell.chute_inner[0] + cell.chute_inner[1]) / 2, (cell.chute_inner[2] + cell.chute_inner[3]) / 2
    z = cell.table_z
    over_nest, in_nest = (nx, ny, z + OVER_DZ), (nx, ny, z + GRIP_DZ)
    over_chute, drop = (cx, cy, z + CHUTE_OVER_DZ), (cx, cy, z + DROP_DZ)
    wps = [Waypoint(1.2, cell.home, FINGER_OPEN, "home")]
    t = 1.2
    for i, tc in enumerate(closes):
        leave = tc - LEAD_S
        if leave < t - 1e-9:
            raise ValueError(f"cycle {i} closes at {tc} s, before the loader can be there")
        if leave > t:
            wps.append(Waypoint(leave - t, cell.home, FINGER_OPEN, f"wait {i}"))
        wps += [
            Waypoint(APPROACH_S, over_nest, FINGER_OPEN, f"to nest {i}"),
            Waypoint(DESCEND_S, in_nest, FINGER_OPEN, f"descend {i}"),
            Waypoint(CLOSE_S, in_nest, FINGER_CLOSED, f"close {i}"),
            Waypoint(LIFT_S, over_nest, FINGER_CLOSED, f"lift {i}"),
            Waypoint(CARRY_S, over_chute, FINGER_CLOSED, f"carry {i}"),
            Waypoint(LOWER_S, drop, FINGER_CLOSED, f"lower {i}"),
            Waypoint(OPEN_S, drop, FINGER_OPEN, f"open {i}"),
            Waypoint(RISE_S, over_chute, FINGER_OPEN, f"rise {i}"),
            Waypoint(RETURN_S, cell.home, FINGER_OPEN, f"return {i}"),
        ]
        t = tc + LIFT_S + CARRY_S + LOWER_S + OPEN_S + RISE_S + RETURN_S
    return wps


@dataclass
class Monitor:
    """What the cell's own sensors would log over one trial; plain NumPy, updated once per control step."""

    cell: Cell
    closes: tuple[float, ...] = ()
    touched: bool = False  # the two arms touched at some point
    double: bool = False  # two blanks lay in the nest at once at some point
    recorded: list = field(default_factory=list)  # per cycle: the supplied blank the loader closed on, or None
    t: float = 0.0

    def reset(self, closes=None) -> None:
        if closes is not None:
            self.closes = tuple(closes)
        self.touched = self.double = False
        self.recorded = [None] * len(self.closes)
        self.t = 0.0

    def in_nest(self, pos) -> bool:
        c = self.cell
        m = c.nest_half + 0.005
        return bool(abs(pos[0] - c.nest_xy[0]) <= m and abs(pos[1] - c.nest_xy[1]) <= m
                    and pos[2] < c.table_z + c.blank_h + 0.005)

    def in_chute(self, pos) -> bool:
        """Centre within the chute's inner walls, and low enough to be resting in it or on the blanks already
        there (under wall height plus one blank)."""
        c = self.cell
        x0, x1, y0, y1 = c.chute_inner
        return bool(x0 <= pos[0] <= x1 and y0 <= pos[1] <= y1 and pos[2] < c.table_z + c.chute_wall_h + c.blank_h)

    def update(self, t: float, blanks: dict, right_tcp, touch: bool) -> None:
        """``blanks``: name -> centre position of every blank (preload included); ``right_tcp``: the controlled
        gripper's tool point; ``touch``: arm-to-arm contact since the last update."""
        self.t = t
        self.touched |= bool(touch)
        inside = [n for n, p in blanks.items() if self.in_nest(p)]
        if len(inside) >= 2:
            self.double = True
        clear = float(np.hypot(*(np.asarray(right_tcp[:2]) - self.cell.nest_xy))) >= self.cell.clear_m
        for i, tc in enumerate(self.closes):
            if self.recorded[i] is None and tc <= t < tc + 0.5:
                self.recorded[i] = "-"  # the loader came; "-": it found no supplied blank it could take
                supplied = [n for n in inside if n != self.cell.preload]
                if len(supplied) == 1 and clear:
                    self.recorded[i] = supplied[0]

    def credited(self, blanks: dict) -> set:
        """Supplied blanks the loader found in the nest at one of its cycles and that are now in the chute."""
        return {n for n in self.recorded if n not in (None, "-") and self.in_chute(blanks[n])}

    def score(self, blanks: dict) -> float:
        if self.touched or self.double:
            return 0.0
        return len(self.credited(blanks)) / len(self.cell.blanks)


# -- Isaac Sim side ------------------------------------------------------------------------------------------------------

_STATE: dict = {}  # cell, monitor, loader plan, runtime


def configure(cell: Cell) -> Monitor:
    mon = Monitor(cell)
    mon.reset(schedule(0, cell))
    _STATE.update(cell=cell, monitor=mon, seed=0)
    return mon


def monitor() -> Monitor:
    return _STATE["monitor"]


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _links(asset) -> list:
    paths = asset.root_physx_view.link_paths
    return list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)


class _Runtime:
    """The arm-to-arm contact view and the right gripper body."""

    def __init__(self, base):
        from isaaclab_physx.physics import PhysxManager

        self.base = base
        right, left = _links(base.scene["right_robot"]), _links(base.scene["left_robot"])
        view = PhysxManager.get_physics_sim_view()
        self.contact = view.create_rigid_contact_view(right, filter_patterns=[left] * len(right),
                                                      max_contact_data_count=64 * len(right) * len(left))
        self.grip = base.scene["right_robot"].find_bodies("gripper")[0][0]
        self.origin = _np(base.scene.env_origins)[0]
        self.dt = float(base.physics_dt)
        self.touch = False
        self.peak = 0.0

    def physics(self) -> None:
        f = _np(self.contact.get_contact_force_matrix(dt=self.dt))
        n = float(np.abs(f).sum())
        if n > 0:
            self.touch = True
            self.peak = max(self.peak, n)

    def right_tcp(self) -> np.ndarray:
        from yam_arena.constants import TCP_OFFSET

        pose = _np(self.base.scene["right_robot"].data.body_link_pose_w)[0, self.grip]
        x, y, z, w = pose[3:7]
        axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        return pose[:3] - self.origin + axis * TCP_OFFSET[2]


def _runtime(base) -> _Runtime:
    rt = _STATE.get("rt")
    if rt is None or rt.base is not base:
        rt = _STATE["rt"] = _Runtime(base)
    return rt


def _new_loader(seed: int):
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    cell = _STATE["cell"]
    closes = schedule(seed, cell)
    _STATE["seed"] = seed
    _STATE["loader"] = ArmPlan(loader_waypoints(closes, cell), station.BASES["left"])
    _STATE["monitor"].reset(closes)
    if _DEBUG:
        print(f"[cell] seed {seed} closes {closes}", flush=True)


def _patch_scene(scene) -> None:
    if getattr(scene, "_cellkit", False):
        return
    object_states = scene.object_states

    def cell_object_states(environment, names):
        out = object_states(environment, names)
        cell, base = _STATE["cell"], environment.unwrapped
        rt = _runtime(base)
        blanks = {n: out[n].pos for n in (*cell.blanks, cell.preload)}
        touch, rt.touch = rt.touch, False
        mon = _STATE["monitor"]
        mon.update(_STATE.get("t", 0.0), blanks, rt.right_tcp(), touch)
        if _DEBUG and round(mon.t * 20) % 20 == 0:
            print(f"[cell] t {mon.t:5.1f} touched {mon.touched} (peak {rt.peak:.2f}) double {mon.double} "
                  f"recorded {mon.recorded} tcp {np.round(rt.right_tcp(), 3)} in nest "
                  f"{[(n, np.round(p, 3).tolist()) for n, p in blanks.items() if mon.in_nest(p)]}", flush=True)
            if round(mon.t * 20) % 200 == 0:
                print(f"[cell] blanks {[(n, np.round(p, 3).tolist()) for n, p in blanks.items()]}", flush=True)
        return out

    scene.object_states, scene._cellkit = cell_object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_cellkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        if "rt" in _STATE and _STATE["rt"].base is base:
            _STATE["rt"].physics()
        return write(*a, **k)

    isc.write_data_to_sim, isc._cellkit = write_data_to_sim, True


class _Command:
    """Stands in for the policy's Action: ``step`` reads only ``.data``."""

    def __init__(self, data):
        self.data = data


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_cellkit", False):
        return
    ensure_sim, reset, step = cls._ensure_sim, cls.reset, cls.step

    def cell_ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
            _runtime(self._base)
        return out

    def cell_reset(self, scene, *, seed=None):
        _new_loader(int(seed or 0))
        _STATE["t"] = 0.0
        out = reset(self, scene, seed=seed)
        rt = _STATE.get("rt")
        if rt is not None:
            rt.touch, rt.peak = False, 0.0  # the settle after env.reset() does not count
        _STATE["monitor"].reset()
        return out

    def cell_step(self, action):
        data = np.asarray(action.data, dtype=np.float64).reshape(-1).copy()
        t = self._t + self._base.step_dt  # where the loader should be at the end of this step
        data[:7] = _STATE["loader"].action7(t)[0]
        _STATE["t"] = t
        return step(self, _Command(data))

    cls._ensure_sim, cls.reset, cls.step, cls._cellkit = cell_ensure_sim, cell_reset, cell_step, True


def _patch_scripted(scripted) -> None:
    """``yam_scripted`` runs the task module's ``demo_controller`` closed loop on each observation."""
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_cellkit", False):
        return
    reset, act = cls.reset, cls.act

    def _controller(self):
        task = getattr(self, "_task", None)
        return getattr(task.module, "demo_controller", None) if task is not None else None

    def cell_reset(self, scene):
        make = _controller(self)
        if make is None:
            return reset(self, scene)
        self._cell_ctl = make(dict(left=self._bases["left"], right=self._bases["right"]), self._dt)
        return None

    def cell_act(self, observation):
        ctl = getattr(self, "_cell_ctl", None)
        if ctl is None or _controller(self) is None:
            return act(self, observation)
        from inspect_robots.types import Action, ActionChunk

        data, done = ctl.act(observation)
        meta = {"request_stop": True} if done else {}
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._cellkit = cell_reset, cell_act, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted}


class _Finder(importlib.abc.MetaPathFinder):
    """Patch each target module right after it is first imported."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _TARGETS:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def _exec(module):
            exec_module(module)
            _TARGETS[fullname](module)

        spec.loader.exec_module = _exec
        return spec


def install() -> None:
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
