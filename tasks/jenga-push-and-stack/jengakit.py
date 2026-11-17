"""jengakit: wooden Jenga blocks and per-arm contact sensing for a yam_arena task file.

yam_arena task files build free boxes with rubber-like friction (1.2) and a 5 mm contact offset. This module adds
:class:`Block`: a free box with its own (wood) friction, a thin contact offset and more solver iterations, so a
54-block tower stands still. Importing the module wraps, without modifying yam_arena:

- ``yam_arena.scene.task_object``: builds a Block as one rigid body with one box collider.
- ``yam_arena.scene.object_states``: every Block is reported as a :class:`BlockState`, which adds whether the left and
  the right arm touch it at the end of the control step (PhysX contact force from any of that arm's links).
- ``yam_arena.inspect_embodiment.YamArenaEmbodiment.reset``: after every reset (and its settle steps) it calls the
  functions in ``ON_RESET``, so the task file can clear its whole-trial latches and take its reference poses.
- ``yam_arena.scripted.YamScriptedPolicy``: if the task module defines ``demo_controller``, ``yam_scripted`` runs it
  closed loop on each observation (joint positions only).

Pure Python at import; Isaac Sim is imported only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

ON_RESET: list = []  # callables run after each embodiment reset
ARMS = ("left_robot", "right_robot")
TOUCH_N = 0.05  # N: a contact force above this counts as touching

# Contact and solver settings for the blocks (checked by the 60 s standing test in the README).
CONTACT_OFFSET = 0.002  # m
REST_OFFSET = 0.0
POS_ITERS = 32
VEL_ITERS = 4
MAX_DEPEN_V = 0.05  # m/s: slow depenetration (faster caps let the loaded tower creep several mm a minute)
SLEEP_THRESHOLD = 0.002  # mass-normalised kinetic energy below which a block sleeps: stops the standing tower's creep
STABILIZATION = 0.001


@dataclass(frozen=True)
class Block(Box):
    """A free wooden block; ``friction`` is (static, dynamic) against everything it touches (combine: average)."""

    friction: tuple[float, float] = (0.40, 0.30)


@dataclass(frozen=True)
class BlockState(ObjectState):
    left: bool = False  # touched by any left-arm link at the end of the step
    right: bool = False


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q) -> float:
    """Angle between the block's own z axis and the world vertical."""
    return math.degrees(math.acos(float(np.clip(quat_matrix(q)[2, 2], -1.0, 1.0))))


# -- Isaac Sim side ---------------------------------------------------------------------------------------------

_SPECS: dict[str, Block] = {}
_PATHS: dict[str, str] = {}  # env 0 prim path of each block
_VIEW: dict = {}  # the PhysX contact view over all blocks, made on the first state read


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _touches(base, names) -> dict[str, tuple[bool, bool]]:
    """(left arm touches, right arm touches) per block, from PhysX's contact force matrix."""
    if "view" not in _VIEW:
        from isaaclab_physx.physics import PhysxManager

        links = {}
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            links[arm] = list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
        order = [n for n in names if n in _PATHS]
        filters = links[ARMS[0]] + links[ARMS[1]]
        _VIEW.update(
            view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                [_PATHS[n] for n in order], filter_patterns=[filters] * len(order), max_contact_data_count=2048),
            order=order, n_left=len(links[ARMS[0]]))
    view, order, nl = _VIEW["view"], _VIEW["order"], _VIEW["n_left"]
    m = _arr(view.get_contact_force_matrix(dt=float(base.physics_dt))).reshape(len(order), -1, 3)
    f = np.linalg.norm(m, axis=2)
    return {n: (bool(f[i, :nl].max(initial=0.0) > TOUCH_N), bool(f[i, nl:].max(initial=0.0) > TOUCH_N))
            for i, n in enumerate(order)}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.block_name]
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(MAX_DEPEN_V)
    rb.CreateSolverPositionIterationCountAttr(POS_ITERS)
    rb.CreateSolverVelocityIterationCountAttr(VEL_ITERS)
    rb.CreateSleepThresholdAttr(SLEEP_THRESHOLD)
    rb.CreateStabilizationThresholdAttr(STABILIZATION)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    _PATHS.setdefault(spec.name, prim_path)
    material = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
    pm.func(material, pm)
    cube = UsdGeom.Cube.Define(stage, f"{prim_path}/geom")
    cube.CreateSizeAttr(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(*spec.size))
    p = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(p)
    col = PhysxSchema.PhysxCollisionAPI.Apply(p)
    col.CreateContactOffsetAttr(CONTACT_OFFSET)
    col.CreateRestOffsetAttr(REST_OFFSET)
    bind_physics_material(str(p.GetPath()), material, stage=stage)
    look = f"{prim_path}/Looks/look"
    vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.6)
    vis.func(look, vis)
    bind_visual_material(str(p.GetPath()), look, stage=stage)
    return prim


def _arena_object(spec: Block):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        block_name: str = ""

    h = spec.yaw / 2
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=_Cfg(block_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(h), math.cos(h))))


def _patch_scene(scene) -> None:
    if getattr(scene, "_jengakit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, Block):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(env, names):
        out = object_states(env, names)
        blocks = [n for n in names if n in _SPECS]
        if blocks:
            touch = _touches(env.unwrapped, blocks)
            for n in blocks:
                out[n] = BlockState(*vars(out[n]).values(), *touch.get(n, (False, False)))
        return out

    scene.task_object, scene.object_states, scene._jengakit = _task_object, _object_states, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_jengakit", False):
        return
    reset = cls.reset

    def _reset(self, scene, *, seed=None):
        obs = reset(self, scene, seed=seed)
        for fn in ON_RESET:
            fn()
        return obs

    cls.reset, cls._jengakit = _reset, True


def _patch_scripted(scripted) -> None:
    """``yam_scripted`` runs the task module's ``demo_controller`` (if any) closed loop on each observation."""
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_jengakit", False):
        return
    reset, act = cls.reset, cls.act

    def _controller(self):
        task = getattr(self, "_task", None)
        return getattr(task.module, "demo_controller", None) if task is not None else None

    def _reset(self, scene):
        make = _controller(self)
        if make is None:
            return reset(self, scene)
        self._jg_ctl = make(dict(left=self._bases["left"], right=self._bases["right"]), self._dt)
        return None

    def _act(self, observation):
        ctl = getattr(self, "_jg_ctl", None)
        if ctl is None or _controller(self) is None:
            return act(self, observation)
        from inspect_robots.types import Action, ActionChunk

        data, done = ctl.act(observation)
        meta = {"request_stop": True} if done else {}
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._jengakit = _reset, _act, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted}


class _Finder(importlib.abc.MetaPathFinder):
    """Patches each target module right after it is first imported."""

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


for _name, _patch in _TARGETS.items():
    if _name in sys.modules:
        _patch(sys.modules[_name])
if not any(isinstance(f, _Finder) for f in sys.meta_path):
    sys.meta_path.insert(0, _Finder())
