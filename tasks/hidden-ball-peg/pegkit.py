"""pegkit: multi-box rigid parts, kit-spawned spheres, and a contact-force probe between two of them, for a
yam_arena task file.

yam_arena task files only take single primitives (Box, Cylinder, Sphere, library assets). This module adds:

- ``Part``: one loose rigid body made of several boxes (a block with a bore in it, a tray) and x-axis cylinders
  (a round peg).
- ``ball``: a sphere spawned with the same solver settings as the parts, with optional angular damping (rolling
  resistance).
- ``PartState``: what ``success()/score()`` get for a Part: the body origin's pose (not a bounding-box centre), its
  velocities and a world bounding box.
- ``contact_force(a, b)``: the magnitude (N) of the contact force between Parts ``a`` and ``b`` in the last physics
  step, from PhysX contact reports (``a`` must be spawned with ``report=True``).
- ``on_physics`` / ``on_reset``: callbacks run with the Isaac Lab env before every physics step (240 Hz) and after
  every reset.

Importing it hooks ``yam_arena.scene`` (spawning and state), ``yam_arena.inspect_embodiment`` (physics step and
reset) and ``yam_arena.demo`` (a ``Waypoint.yaw`` may be a 3x3 world rotation matrix, so ``yam_scripted`` can play
grips other than straight down). yam_arena itself is not modified. Pure Python at import; Isaac Sim is imported
lazily. Adapted from the jointkit module of an earlier task (hinges and joints removed, contact probe added).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

Color = tuple[float, float, float]
BoxSpec = tuple  # (centre, size (m), sRGB colour), or (centre, (length, d, d), colour, "cyl"): a cylinder of diameter d
#                 along the part's x axis
PHYSICS_DT = 1 / 240


@dataclass(frozen=True)
class Part(Box):
    """A loose rigid body made of ``boxes`` (part frame, origin at ``pos``), or one sphere (``radius`` > 0).
    Subclasses Box so yam_arena's task-file checks accept it; ``size`` is the boxes' extent and ``lo``/``hi`` its
    corners in the part frame. ``rot`` (3x3, row-major tuple) overrides ``yaw`` for the initial orientation."""

    boxes: tuple[BoxSpec, ...] = ()
    friction: float = 0.9
    lo: tuple[float, float, float] = (0.0, 0.0, 0.0)
    hi: tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 0.0
    angular_damping: float = 0.05
    report: bool = False  # PhysX contact reports on (needed for contact_force(name, ...))

    def _check(self) -> None:
        super()._check()
        if not (self.boxes or self.radius > 0) or self.mass is None:
            raise ValueError(f"{self.name}: a Part needs boxes (or a radius) and a mass")


def part(name, pos, boxes, mass, yaw=0.0, color=(0.5, 0.5, 0.5), friction=0.9, report=False) -> Part:
    lo = np.min([np.subtract(b[0], np.divide(b[1], 2)) for b in boxes], axis=0)
    hi = np.max([np.add(b[0], np.divide(b[1], 2)) for b in boxes], axis=0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), friction=friction, report=report, lo=tuple(lo.tolist()), hi=tuple(hi.tolist()))


def ball(name, pos, radius, mass, color=(0.5, 0.5, 0.5), friction=0.5, angular_damping=0.05,
         report=False) -> Part:
    """A sphere, spawned with the parts' solver settings (32 position iterations, 1 m/s depenetration)."""
    r = float(radius)
    return Part(name=name, pos=tuple(pos), size=(2 * r,) * 3, mass=mass, color=color, friction=friction,
                lo=(-r, -r, -r), hi=(r, r, r), radius=r, angular_damping=angular_damping, report=report)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


@dataclass(frozen=True)
class PartState(ObjectState):
    """A Part after a control step: body origin pose, velocities, world bounding box."""

    R: np.ndarray = field(default=None, repr=False)

    def rotation(self) -> np.ndarray:
        return quat_to_matrix(self.quat_xyzw) if self.R is None else self.R

    def to_local(self, p) -> np.ndarray:
        return self.rotation().T @ (np.asarray(p, float) - self.pos)

    def to_world(self, p) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(p, float)

    def tilt(self) -> float:
        """Angle (rad) between the part's z axis and the vertical."""
        return math.acos(max(-1.0, min(1.0, float(self.rotation()[2, 2]))))


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}
_PATHS: dict[str, str] = {}  # part name -> its body prim path in env 0


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    material = f"{prim_path}/Looks/physics"
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    mat.func(material, mat)
    looks: dict = {}

    path = f"{prim_path}/body"
    _PATHS.setdefault(spec.name, path)
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateAngularDampingAttr(spec.angular_damping)
    if spec.report:
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    shapes = list(enumerate(spec.boxes)) + ([(-1, ((0, 0, 0), None, spec.color))] if spec.radius > 0 else [])
    for i, (centre, size, color, *kind) in shapes:
        if size is None:
            geom = UsdGeom.Sphere.Define(stage, f"{path}/sphere")
            geom.CreateRadiusAttr(spec.radius)
        elif kind == ["cyl"]:
            geom = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            geom.CreateAxisAttr("X")
            geom.CreateHeightAttr(size[0])
            geom.CreateRadiusAttr(size[1] / 2)
            geom.AddTranslateOp().Set(Gf.Vec3d(*centre))
        else:
            geom = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*centre))
            geom.AddScaleOp().Set(Gf.Vec3f(*size))
        c = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(c)
        col = PhysxSchema.PhysxCollisionAPI.Apply(c)
        col.CreateContactOffsetAttr(0.001)
        col.CreateRestOffsetAttr(0.0)
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(looks[color], vis)
        bind_physics_material(str(c.GetPath()), material, stage=stage)
        bind_visual_material(str(c.GetPath()), looks[color], stage=stage)
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        part_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=_Cfg(part_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos),
                                    rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


# -- reading state ------------------------------------------------------------------------------------------------


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    r = quat_to_matrix(pose[3:7])
    corners = np.array([[a, b, c] for a in (spec.lo[0], spec.hi[0]) for b in (spec.lo[1], spec.hi[1])
                        for c in (spec.lo[2], spec.hi[2])]) @ r.T + pos
    return PartState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=corners.min(0),
                     high=corners.max(0), R=r)


_VIEWS: dict = {}


def contact_force(a: str, b: str) -> float:
    """|contact force| (N) between Parts ``a`` (spawned with ``report=True``) and ``b`` in the last physics step
    (env 0), from PhysX's contact report force matrix."""
    if (a, b) not in _VIEWS:
        from isaaclab_physx.physics import PhysxManager

        view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            _PATHS[a], filter_patterns=[_PATHS[b]], max_contact_data_count=64)
        _VIEWS[(a, b)] = view
    m = _VIEWS[(a, b)].get_contact_force_matrix(dt=PHYSICS_DT)
    m = m.numpy() if hasattr(m, "numpy") else _np(m)
    return float(np.linalg.norm(np.asarray(m, float).reshape(-1, 3).sum(0)))


# -- hooks ------------------------------------------------------------------------------------------------------

_PHYSICS: list = []
_RESET: list = []


def on_physics(fn):
    _PHYSICS.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_pegkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        out = object_states(environment, [n for n in names if n not in _SPECS])
        out.update({n: _part_state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._pegkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    """The env calls scene.write_data_to_sim() once before every physics substep."""
    isc = environment.unwrapped.scene
    if getattr(isc, "_pegkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for fn in _PHYSICS:
            fn(environment)
        return write(*a, **k)

    isc.write_data_to_sim, isc._pegkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_pegkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, *a, **k):
        for fn in _RESET:  # before env.reset(): the settle steps after it already run with fresh state
            fn()
        return reset(self, *a, **k)

    cls._ensure_sim, cls.reset, cls._pegkit = _ensure_sim, _reset, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_pegkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._pegkit = _grip_rotation, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.demo": _patch_demo}


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


def install() -> None:
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
