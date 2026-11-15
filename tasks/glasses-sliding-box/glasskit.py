"""glasskit: compound rigid objects and a per-trial reset hook, for a yam_arena task file.

yam_arena task files take single primitives only. This module adds :class:`Compound`: one rigid body (free, with a
mass) or one fixed collider (no mass) made of several boxes in its own frame, each with its own friction; used here
for the glasses, the keys, the lid and the box with its guides. Importing the module wraps,
without modifying yam_arena:

- ``yam_arena.scene.task_object``: builds a Compound as an Arena object whose root prim is the rigid body, with one
  box collider child per part.
- ``yam_arena.scene.object_states``: for a Compound, ``ObjectState.pos`` is the origin of its own frame (not the
  bounding-box centre that yam_arena reports for primitives).
- ``yam_arena.inspect_embodiment.YamArenaEmbodiment.reset``: after every reset (and its settle steps) it calls the
  functions in ``ON_RESET``, so a task file can clear its whole-trial latches.

Pure Python at import; Isaac Sim is imported only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box

ON_RESET: list = []  # callables run after each embodiment reset


@dataclass(frozen=True)
class Compound(Box):
    """A rigid body made of ``parts``: (centre, size, sRGB colour) boxes in the body frame (m). ``pos`` is the frame's
    origin in the world; ``size`` is the parts' bounding box (filled in by :func:`compound`)."""

    parts: tuple = ()
    friction: tuple[float, float] | None = None  # (static, dynamic); None: yam_arena's grippy default


def compound(name, pos, parts, mass, yaw=0.0, friction=None) -> Compound:
    lo = np.min([np.subtract(c, np.divide(s, 2)) for c, s, _ in parts], axis=0)
    hi = np.max([np.add(c, np.divide(s, 2)) for c, s, _ in parts], axis=0)
    return Compound(name=name, pos=tuple(pos), mass=mass, yaw=yaw, size=tuple(float(v) for v in hi - lo),
                    parts=tuple(parts), friction=friction)


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q) -> float:
    """Angle between the body's z axis and the world vertical."""
    return math.degrees(math.acos(float(np.clip(quat_matrix(q)[2, 2], -1.0, 1.0))))


def corners(spec: Compound, pos, q) -> np.ndarray:
    """World positions of every corner of every part (8 per part)."""
    R = quat_matrix(q)
    pts = [np.add(c, np.multiply(s, [sx, sy, sz]) / 2) for c, s, _ in spec.parts
           for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return np.asarray(pos) + np.asarray(pts) @ R.T


# -- Isaac Sim side ---------------------------------------------------------------------------------------------

_SPECS: dict[str, Compound] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.compound_name]
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    prim = root.GetPrim()
    if spec.free:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(16)
    mu = spec.friction or GRIPPY_FRICTION
    material = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=mu[0], dynamic_friction=mu[1],
                                        friction_combine_mode="min")
    pm.func(material, pm)
    looks: dict = {}
    for i, (centre, size, color) in enumerate(spec.parts):
        cube = UsdGeom.Cube.Define(stage, f"{prim_path}/part_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        p = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        col = PhysxSchema.PhysxCollisionAPI.Apply(p)
        col.CreateContactOffsetAttr(0.003)
        col.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(p.GetPath()), material, stage=stage)
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.35)
            vis.func(looks[color], vis)
        bind_visual_material(str(p.GetPath()), looks[color], stage=stage)
    return prim


def _arena_object(spec: Compound):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        compound_name: str = ""

    h = spec.yaw / 2
    return Object(name=spec.name, object_type=ObjectType.RIGID if spec.free else ObjectType.BASE, spawner_cfg=_Cfg(compound_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(h), math.cos(h))))


def _patch_scene(scene) -> None:
    if getattr(scene, "_glasskit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, Compound):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(env, names):
        out = object_states(env, names)
        base = env.unwrapped
        origin = base.scene.env_origins[0]
        for n in names:
            if n in _SPECS:
                p = (base.arena_world.get_pose_w(n)[0, :3] - origin).detach().cpu().numpy().astype(np.float64)
                s = out[n]
                out[n] = type(s)(p, s.quat_xyzw, s.lin_vel, s.ang_vel, s.low, s.high)
        return out

    scene.task_object, scene.object_states, scene._glasskit = _task_object, _object_states, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_glasskit", False):
        return
    reset = cls.reset

    def _reset(self, scene, *, seed=None):
        obs = reset(self, scene, seed=seed)
        for fn in ON_RESET:
            fn()
        return obs

    cls.reset, cls._glasskit = _reset, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


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
