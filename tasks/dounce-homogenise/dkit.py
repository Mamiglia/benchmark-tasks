"""dkit: a glass Dounce tissue grinder (mortar and tight pestle) for a yam_arena task file.

yam_arena task files only take rigid primitives. This module adds one object, :class:`DounceGrinder`, spawned as a
FLOATING-base articulation:

- ``mortar`` (root): the 13 mm grinding tube with its glass floor and the 20 mm reservoir on top (colliders), and
  the visible sample: a column below the head, a column above it in the annulus round the shaft, and a foam band.
- ``pestle``: head, 6 mm shaft and knob (colliders on the shaft and knob, which the fingers grip), on a PRISMATIC
  joint along the bore's axis. The 0.05 mm fit lets the head only slide along the bore, so the joint stands in for
  the fit. The liquid's resistance to the head is the joint's damping (an implicit drive, ``dounce.DRAG`` N s/m,
  no stiffness), so it acts on the pestle and, equal and opposite, on the mortar: a pull lifts an unheld mortar.

After every control step ``yam_arena.scene.object_states`` reads the joint and the mortar's pose and steps
:class:`dounce.Dounce` (strokes, air, cavitation, foam, latches). Importing the module also hooks
``yam_arena.scripted``: when the task file defines ``controller(bases, dt)``, ``yam_scripted`` asks that object for
every action, passing it the observation. yam_arena itself is not modified. Pure Python at import.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import dounce as D  # noqa: E402

ARMS = ("left_robot", "right_robot")
TCP_OFFSET = 0.134
_DEBUG = bool(os.environ.get("DOUNCE_DEBUG"))

# mortar (origin: the outside of its bottom), metres
FLOOR = 0.002  # glass floor thickness
TUBE_R = 0.0065
RES_R, RES_H = 0.0100, 0.022  # reservoir on top of the grinding tube
MORTAR_H = FLOOR + D.TUBE_L + RES_H
# pestle (origin: the head's bottom face)
KNOB_R, KNOB_Z0, KNOB_Z1 = 0.0070, 0.160, 0.175
GRIP_Z = (KNOB_Z0 + KNOB_Z1) / 2  # the demo grips the knob

PATCH_R = 0.004  # m: contact patch of a finger pad (with a nitrile cot) on the glass
GLASS = (0.80, 0.86, 0.90)
SAMPLE = (0.93, 0.80, 0.55)  # pale buff cell suspension
FOAM = (0.99, 0.99, 0.99)


@dataclass(frozen=True)
class DounceGrinder(Box):
    """``pos``: the outside of the mortar's bottom, centre (env frame)."""

    mortar_mass: float = 0.035  # 7 mL borosilicate mortar (~30 g) with ~3 mL of sample
    pestle_mass: float = 0.015
    friction: float = 1.0
    upper: float = 0.085  # joint travel: head bottom from the floor up into the reservoir


@dataclass(frozen=True)
class DounceState(ObjectState):
    z: float = 0.0  # head bottom above the floor
    zdot: float = 0.0
    lift: float = 0.0
    tilt: float = 0.0
    axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    model: D.Dounce = field(default_factory=D.Dounce)
    tcps: dict = field(default_factory=dict)


_SPECS: dict[str, DounceGrinder] = {}
_PATHS: dict[str, str] = {}
_MODEL = D.Dounce()
_LOOKS: dict = {}


def model() -> D.Dounce:
    return _MODEL


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------------


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as su
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    s = _SPECS[cfg.obj_name]
    _PATHS[s.name] = prim_path
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    mat = f"{prim_path}/Looks/phys"
    m = su.RigidBodyMaterialCfg(static_friction=s.friction, dynamic_friction=s.friction)
    m.func(mat, m)
    looks: dict = {}

    def look(rgb, opacity=1.0):
        key = (tuple(rgb), opacity)
        if key not in looks:
            looks[key] = f"{prim_path}/Looks/c{len(looks)}"
            v = su.PreviewSurfaceCfg(diffuse_color=linear_rgb(rgb), roughness=0.2, opacity=opacity)
            v.func(looks[key], v)
        return looks[key]

    def cyl(path, r, z0, z1, rgb, solid, opacity=1.0):
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(r)
        c.CreateHeightAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, (z0 + z1) / 2))
        c.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, max(z1 - z0, 1e-5)))
        prim = c.GetPrim()
        if solid:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            cp.CreateTorsionalPatchRadiusAttr(PATCH_R)  # padded fingers on glass also resist twisting
            cp.CreateMinTorsionalPatchRadiusAttr(PATCH_R)
            bind_physics_material(str(prim.GetPath()), mat, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(rgb, opacity), stage=stage)
        return c

    def body(name, pos, mass, com, inertia):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr(mass)
        api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, com))
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(1.0)
        return path

    h = MORTAR_H
    mor = body("mortar", (0.0, 0.0, 0.0), s.mortar_mass, h / 2,
               (s.mortar_mass * h * h / 12, s.mortar_mass * h * h / 12, s.mortar_mass * TUBE_R**2))
    top = FLOOR + D.TUBE_L
    cyl(f"{mor}/tube", TUBE_R, 0.0, top, GLASS, True, 0.35)
    cyl(f"{mor}/reservoir", RES_R, top, h, GLASS, True, 0.30)
    cyl(f"{mor}/lip", RES_R + 0.0008, h - 0.002, h, GLASS, False)
    lo = cyl(f"{mor}/liq_low", TUBE_R + 0.0002, FLOOR, FLOOR + 1e-4, SAMPLE, False)
    hi = cyl(f"{mor}/liq_high", TUBE_R + 0.0002, FLOOR, FLOOR + 1e-4, SAMPLE, False, 0.85)
    fo = cyl(f"{mor}/foam", TUBE_R + 0.0004, FLOOR, FLOOR + 1e-4, FOAM, False)
    UsdGeom.Imageable(fo.GetPrim()).MakeInvisible()
    _LOOKS[s.name] = {"low": lo, "high": hi, "foam": fo}

    L = KNOB_Z1
    pes = body("pestle", (0.0, 0.0, FLOOR), s.pestle_mass, 0.10,
               (s.pestle_mass * L * L / 12, s.pestle_mass * L * L / 12, s.pestle_mass * SHAFT_R2))
    cyl(f"{pes}/head", D.BORE_R - 0.0001, 0.0, D.HEAD_L, (0.86, 0.90, 0.92), False)
    cyl(f"{pes}/shaft", D.SHAFT_R, D.HEAD_L, KNOB_Z0, (0.84, 0.88, 0.91), True)
    cyl(f"{pes}/knob", KNOB_R, KNOB_Z0, KNOB_Z1, (0.84, 0.88, 0.91), True)

    j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/pestle_joint")
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(mor)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(pes)])
    j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, FLOOR))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(s.upper)
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(0.0)
    d.CreateDampingAttr(D.DRAG)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(mor))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(mor))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


SHAFT_R2 = D.SHAFT_R**2


def _arena_object(spec: DounceGrinder):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        obj_name: str = ""

    class _Obj(Object):
        def _generate_articulation_cfg(self):
            act = {"pestle": ImplicitActuatorCfg(joint_names_expr=["pestle_joint"], stiffness=0.0, damping=D.DRAG)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=act)
            return self._add_initial_pose_to_cfg(cfg)

    return _Obj(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=_Cfg(obj_name=spec.name),
                initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# -- runtime -----------------------------------------------------------------------------------------------------------


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


class _Runtime:
    def __init__(self, base, name):
        self.base, self.name = base, name
        self.origin = _np(base.scene.env_origins)[0]
        self.grip = {a: base.scene[a].find_bodies("gripper")[0][0] for a in ARMS}
        self.seat_z = float(_SPECS[name].pos[2])
        self.shown = None

    def tcps(self):
        out = {}
        for a in ARMS:
            p = _np(self.base.scene[a].data.body_link_pose_w)[0, self.grip[a]]
            out[a] = p[:3] - self.origin + quat_to_matrix(p[3:7])[:, 2] * TCP_OFFSET
        return out

    def show(self, m: D.Dounce, z: float) -> None:
        from pxr import Gf, UsdGeom

        lv = D.level(m.v_ml, z)
        foam = m.foam_mm >= D.FOAM_OK_MM
        key = (round(z, 4), round(lv, 4), foam)
        if key == self.shown:
            return
        self.shown = key
        try:
            lk = _LOOKS[self.name]
            low_top = min(z, lv)
            for prim, z0, z1 in ((lk["low"], 0.0, low_top), (lk["high"], z + D.HEAD_L, lv)):
                ops = prim.GetOrderedXformOps()
                hgt = max(z1 - z0, 1e-5)
                ops[0].Set(Gf.Vec3d(0.0, 0.0, FLOOR + z0 + hgt / 2))
                ops[1].Set(Gf.Vec3f(1.0, 1.0, hgt if z1 > z0 else 1e-5))
            ops = lk["foam"].GetOrderedXformOps()
            fh = max(m.foam_mm, 2.0) * 1e-3
            ops[0].Set(Gf.Vec3d(0.0, 0.0, FLOOR + lv + fh / 2))
            ops[1].Set(Gf.Vec3f(1.0, 1.0, fh))
            img = UsdGeom.Imageable(lk["foam"].GetPrim())
            img.MakeVisible() if foam else img.MakeInvisible()
        except Exception as err:  # cosmetic only
            if _DEBUG:
                print(f"[dkit] show failed: {err}", flush=True)

    def state(self) -> DounceState:
        a = self.base.scene[self.name]
        q = float(_np(a.data.joint_pos)[0, 0])
        qd = float(_np(a.data.joint_vel)[0, 0])
        pose = _np(a.data.root_link_pose_w)[0]
        vel = _np(a.data.root_vel_w)[0]
        pos = pose[:3] - self.origin
        axis = quat_to_matrix(pose[3:7])[:, 2]
        tilt = math.degrees(math.acos(float(np.clip(axis[2], -1, 1))))
        lift = float(pos[2] - self.seat_z)
        m = _MODEL
        m.step(q, lift, tilt, float(self.base.step_dt))
        self.show(m, q)
        if _DEBUG and round(m.t / float(self.base.step_dt)) % 10 == 0:
            print(f"[dkit] t {m.t:5.1f} z {q * 1000:5.1f} mm v {qd * 1000:6.1f} mm/s lift {lift * 1000:4.1f} "
                  f"tilt {tilt:4.1f} strokes {m.strokes} air {m.air} cav {m.cav} pos {np.round(pos * 1000, 1)} "
                  f"axis {np.round(axis, 3)} tcpR {np.round(self.tcps()['right_robot'] * 1000, 1)}", flush=True)
        top = pos + axis * (FLOOR + q + KNOB_Z1)
        return DounceState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                           low=pos - (RES_R, RES_R, 0.0), high=np.maximum(pos + axis * MORTAR_H, top) + RES_R,
                           z=q, zdot=qd, lift=lift, tilt=tilt, axis=axis, model=m, tcps=self.tcps())


_RT: dict = {}


def _runtime(base, name) -> _Runtime:
    rt = _RT.get(name)
    if rt is None or rt.base is not base:
        rt = _RT[name] = _Runtime(base, name)
    return rt


# -- hooks -------------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_dkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, DounceGrinder):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._dkit = _task_object, _object_states, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_dkit", False):
        return
    reset = cls.reset

    def _reset(self, scene, *, seed=None):
        obs = reset(self, scene, seed=seed)
        v = D.volume_for(seed)
        _MODEL.reset(v)  # the settle steps after env.reset() are not part of the trial
        for rt in _RT.values():
            rt.shown = None
        print(f"[dkit] seed {seed}: sample {v:.2f} mL, air above head height {D.z_air(v) * 1000:.1f} mm", flush=True)
        return obs

    cls.reset, cls._dkit = _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_dkit", False):
        return
    reset, act = cls.reset, cls.act

    def _make(self):
        task = self._task
        make = getattr(task.module, "controller", None) if task is not None else None
        return make(dict(self._bases), self._dt) if make is not None else None

    def _reset(self, scene):
        self._ctl = _make(self)
        if self._ctl is None:
            return reset(self, scene)

    def _act(self, observation):
        if getattr(self, "_ctl", None) is None:
            return act(self, observation)
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack

        left, right, finished_at = self._ctl.act(observation)
        stop = finished_at is not None and float(observation.state_time) > finished_at + self.hold_s
        return ActionChunk(actions=[Action(data=pack(left, right), meta={"request_stop": True} if stop else {})],
                           control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._dkit = _reset, _act, True


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


def install() -> None:
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
