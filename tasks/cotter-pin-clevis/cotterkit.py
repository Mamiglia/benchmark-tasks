"""cotterkit: a fixed clevis joint whose cotter pin has two plastically bending legs, for a yam_arena task file.

yam_arena task files only take rigid primitives. This module adds one object, :class:`CotterRig`, and hooks
yam_arena (without modifying it) so that:

- ``yam_arena.scene.task_object`` builds it as a FIXED-base articulation. The root body is the clevis joint (stand,
  fork, rod end, clevis pin, washer) with the cotter pin's eye and shank, which sit in the pin's cross-hole. Its two
  children are the cotter's legs, each on a revolute joint about the vertical at its root, where it leaves the
  cross-hole, free to bend either way (+-100 degrees). The legs lie flat against each other at the start and
  collide with each other, so a leg pushed toward the other carries it along (both legs can end on one side).
- Before every PHYSICS step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) a :class:`Leg` per leg
  turns the measured joint angle into the leg's permanent set (elastic-plastic bending, below) and writes it as the
  joint's position target. The joint's implicit drive has the leg's elastic bending stiffness, so the moment it
  takes is ``k (angle - set)``; once that reaches the yield moment the set follows the leg. A leg pushed under its
  yield moment springs back; pushed past it, it stays bent, less a small springback.
- ``yam_arena.scene.object_states`` reports a :class:`CotterState`: both legs' bend and set, the latches, the
  legs' angular speed and the grippers' tool points.

The top half (``LegParams``, ``Leg``, the geometry helpers) is plain Python and NumPy; the tests exercise it.
Everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

LEGS = ("long", "short")
JOINTS = {"long": "long_leg_bend", "short": "short_leg_bend"}
SIGN = {"long": 1.0, "short": -1.0}  # joint angle (counter-clockwise about +z) = SIGN * bend; long leg at +y
ARMS = ("left_robot", "right_robot")
TCP_OFFSET = 0.134  # yam_arena.constants.TCP_OFFSET, on the gripper link's z axis
STEEL = 7850.0  # kg/m^3


# -- the legs' mechanics (pure Python) ----------------------------------------------------------------------------


def semicircle_plastic_modulus(r: float) -> float:
    """Plastic section modulus (m^3) of a half-round wire of radius ``r``, bending about an axis parallel to its flat
    face (how a cotter pin's legs bend when they are spread apart). Numerical: the plastic neutral axis halves the
    area."""
    y = np.linspace(0.0, r, 20001)
    w = 2.0 * np.sqrt(np.maximum(r * r - y * y, 0.0))
    da = w * (y[1] - y[0])
    c = np.cumsum(da)
    yp = y[np.searchsorted(c, c[-1] / 2)]
    return float(np.sum(np.abs(y - yp) * da))


def semicircle_second_moment(r: float) -> float:
    """Second moment of area (m^4) of a half-round about its centroidal axis parallel to the flat face."""
    return (math.pi / 8 - 8 / (9 * math.pi)) * r**4


@dataclass(frozen=True)
class LegParams:
    """One cotter-pin leg: a half-round of ``wire_d`` (the pin's actual diameter, both legs together) of
    low-carbon steel. Lengths in m, stresses in Pa, angles in degrees."""

    wire_d: float = 0.0018  # ISO 1234 2 x 25: d 1.7-1.8 mm actual
    yield_pa: float = 300e6  # annealed low-carbon steel wire (README: how to check it on the real pins)
    hardening: float = 0.25  # the yield moment rises 25% over the first 90 degrees of set (cold work)
    youngs_pa: float = 200e9
    stiff_len: float = 0.017  # the elastic stiffness is that of a cantilever of this length (the short leg)
    rebend_after_deg: float = 30.0  # a leg once set this far ...
    rebend_deg: float = 15.0  # ... and then bent back this much is a re-bent (scrap) cotter
    damping: float = 2e-5  # N m s/rad, implicit joint damping
    armature: float = 1e-7  # kg m^2

    @property
    def yield_nm(self) -> float:
        """Plastic (fully yielded) bending moment at the root, N m."""
        return self.yield_pa * semicircle_plastic_modulus(self.wire_d / 2)

    @property
    def stiffness(self) -> float:
        """Elastic bending stiffness at the root, N m/rad: a cantilever loaded at its end, 3 E I / L."""
        return 3 * self.youngs_pa * semicircle_second_moment(self.wire_d / 2) / self.stiff_len

    def moment_nm(self, peak_set_rad: float) -> float:
        """Yield moment after a peak set of ``peak_set_rad`` (linear hardening)."""
        return self.yield_nm * (1.0 + self.hardening * min(peak_set_rad, math.pi) / (math.pi / 2))

    def force_n(self, lever_m: float, peak_set_rad: float = 0.0) -> float:
        """Side force at ``lever_m`` from the root that yields the leg."""
        return self.moment_nm(peak_set_rad) / lever_m


class Leg:
    """A leg's permanent set (rad, bend sense: 0 straight, + away from the other leg, - toward it) and its latches.
    ``peak`` is the set furthest from straight (signed). Call
    :meth:`step` with the measured bend every physics step; it returns the new set (the joint's drive target) and
    the moment the leg carries (N m)."""

    def __init__(self, params: LegParams = LegParams()):
        self.p = params
        self.reset()

    def reset(self) -> None:
        self.set = 0.0
        self.peak = 0.0
        self.rebent = False
        self.peak_moment = 0.0

    def step(self, bend: float) -> tuple[float, float]:
        p = self.p
        k, m_y = p.stiffness, p.moment_nm(abs(self.peak))
        m = k * (bend - self.set)
        if abs(m) > m_y:  # yielding (either way): the set follows the leg
            m = math.copysign(m_y, m)
            self.set = bend - m / k
        if abs(self.set) > abs(self.peak):
            self.peak = self.set
        self.peak_moment = max(self.peak_moment, abs(m))
        # bent back toward (or through) straight by rebend_deg from the furthest set, once that was rebend_after_deg
        far = abs(self.peak) >= math.radians(p.rebend_after_deg)
        if far and abs(self.set - self.peak) >= math.radians(p.rebend_deg):
            self.rebent = True
        return self.set, m


# -- the task-file object ------------------------------------------------------------------------------------------

# (centre, size, colour[, "cyl"]) in the rig frame (m); "cyl" draws and collides an upright cylinder of diameter
# size[0] and height size[2] instead of a box.
BoxSpec = tuple


@dataclass(frozen=True)
class CotterRig(Box):
    """A fixed clevis joint. ``pos`` is the rig frame's origin: the clevis pin's axis at the height of the cotter's
    axis. The cotter lies along +x (eye toward -x, the robot); its legs leave the pin at ``root_x`` and lie side by
    side, the long one on the +y side. ``boxes`` are the fixed parts (clevis and the cotter's eye)."""

    boxes: tuple[BoxSpec, ...] = ()
    root_x: float = 0.004  # the pin's far surface
    long_len: float = 0.021
    short_len: float = 0.017
    leg_t: float = 0.0009  # each leg's thickness (in the bend direction, y) ...
    leg_w: float = 0.0018  # ... and width (z)
    leg_color: tuple[float, float, float] = (0.78, 0.78, 0.74)
    max_bend_deg: float = 100.0  # a leg wrapped round the pin
    friction: float = 0.6  # steel parts
    sim_leg_mass: float = 0.02  # kg per leg in the solver (real: 0.17-0.21 g): a 0.2 g link against the PD-driven
    # finger gives PhysX too weak a contact, and the finger passes through the leg. The legs turn only about the
    # vertical, so their weight does nothing; at the demo's few mm/s their inertia adds well under 0.01 N.
    params: LegParams = LegParams()

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a CotterRig is fixed (mass None)")

    def leg_len(self, leg: str) -> float:
        return self.long_len if leg == "long" else self.short_len

    def leg_mass(self, leg: str) -> float:
        return STEEL * math.pi * (self.leg_w / 2) ** 2 / 2 * self.leg_len(leg)


def leg_tip(spec: CotterRig, leg: str, bend_rad: float) -> np.ndarray:
    """A leg's tip in the rig frame (x, y) at ``bend_rad``."""
    a = SIGN[leg] * bend_rad
    return np.array([spec.root_x + spec.leg_len(leg) * math.cos(a), spec.leg_len(leg) * math.sin(a)])


def point_segment_distance(p, a, b) -> float:
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(ab @ ab, 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


@dataclass(frozen=True)
class CotterState(ObjectState):
    """After a control step. ``pos`` is the rig origin (env frame). Angles in degrees, bend sense."""

    bend_deg: dict = field(default_factory=dict)  # leg -> measured bend
    set_deg: dict = field(default_factory=dict)  # leg -> permanent set
    peak_set_deg: dict = field(default_factory=dict)
    rebent: dict = field(default_factory=dict)  # leg -> latched
    leg_speed: float = 0.0  # fastest leg, deg/s
    gap_m: float = 1.0  # nearest finger pad tip (or tool point) to either leg or the eye
    peak_moment_nm: dict = field(default_factory=dict)


# -- Isaac Sim side ------------------------------------------------------------------------------------------------

_SPECS: dict[str, CotterRig] = {}
_LEGS: dict[str, dict[str, Leg]] = {}
_DEBUG = [0] if os.environ.get("COTTER_DEBUG") else []


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.rig_name]
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    mat = f"{prim_path}/Looks/steel"
    m = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    m.func(mat, m)
    looks: dict = {}

    def look(color) -> str:
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.35, metallic=0.5)
            vis.func(looks[color], vis)
        return looks[color]

    def body(name: str, pos, mass: float) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        return path

    def shapes(path: str, specs, offset: float) -> None:
        for i, (centre, size, color, *kind) in enumerate(specs):
            if kind and kind[0] == "cyl":
                g = UsdGeom.Cylinder.Define(stage, f"{path}/s{i}")
                g.CreateRadiusAttr(size[0] / 2)
                g.CreateHeightAttr(size[2])
                g.CreateAxisAttr("Z")
                g.AddTranslateOp().Set(Gf.Vec3d(*centre))
            else:
                g = UsdGeom.Cube.Define(stage, f"{path}/s{i}")
                g.CreateSizeAttr(1.0)
                g.AddTranslateOp().Set(Gf.Vec3d(*centre))
                g.AddScaleOp().Set(Gf.Vec3f(*size))
            prim = g.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(offset)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(prim.GetPath()), mat, stage=stage)
            bind_visual_material(str(prim.GetPath()), look(tuple(color)), stage=stage)

    clevis = body("clevis", (0.0, 0.0, 0.0), 1.0)
    shapes(clevis, spec.boxes, 0.002)
    for leg in LEGS:
        L, s = spec.leg_len(leg), SIGN[leg]
        path = body(f"{leg}_leg", (spec.root_x, 0.0, 0.0), spec.sim_leg_mass)
        shapes(path, [((L / 2, s * spec.leg_t / 2, 0.0), (L, spec.leg_t, spec.leg_w), spec.leg_color)], 0.0008)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{JOINTS[leg]}")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(clevis)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(spec.root_x, 0.0, 0.0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        lo, hi = -spec.max_bend_deg, spec.max_bend_deg  # either way: the scorer, not a limit, decides the side
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")  # gains come from the actuator config
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)

    # Fixed base: a fixed joint from the world to the clevis body, the articulation root on the parent Xform (the
    # PhysX parser treats a fixed joint on a rigid-body root as a maximal-coordinate joint otherwise).
    fj = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/world_fixed")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(clevis)])
    t = translation or (0.0, 0.0, 0.0)
    fj.CreateLocalPos0Attr(Gf.Vec3f(*t))
    fj.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(True)  # the legs push each other (a leg and the clevis body are filtered)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return root.GetPrim()


def _arena_object(spec: CotterRig):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    p = spec.params

    @configclass
    class CotterSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        rig_name: str = ""

    class CotterObject(Object):
        def _generate_articulation_cfg(self):
            # The legs' elastic stiffness as an implicit drive; its target (the permanent set) comes from _apply().
            actuators = {
                "legs": ImplicitActuatorCfg(
                    joint_names_expr=list(JOINTS.values()), stiffness=p.stiffness, damping=p.damping,
                    armature=p.armature,
                )
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return CotterObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=CotterSpawnerCfg(rig_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)  # Isaac Lab 3.0 data are warp arrays with a .torch view
    return x.detach().cpu().numpy().astype(np.float64)


def _bends(base, spec: CotterRig):
    asset = base.scene[spec.name]
    names = list(asset.joint_names)
    ids = {leg: names.index(JOINTS[leg]) for leg in LEGS}
    q = _np(asset.data.joint_pos)[0]
    dq = _np(asset.data.joint_vel)[0]
    return asset, ids, {leg: SIGN[leg] * q[ids[leg]] for leg in LEGS}, {leg: abs(dq[ids[leg]]) for leg in LEGS}


def _apply(base) -> None:
    import torch

    for name, spec in _SPECS.items():
        asset, ids, bend, _ = _bends(base, spec)
        target = torch.zeros((1, asset.num_joints), dtype=torch.float32, device=base.device)
        for leg in LEGS:
            s, _ = _LEGS[name][leg].step(bend[leg])
            target[0, ids[leg]] = SIGN[leg] * s
        asset.set_joint_position_target_index(target=target)


PAD_HALF_W = 0.008  # a finger's tip is 4 mm thick and 15.4 mm wide: points stand for it within 8 mm


def _tool_points(base) -> list[np.ndarray]:
    """Points along each arm's finger tips (env frame): the tool point, and for each finger its tip centre and the
    centre of its lowest 13 mm. Distances to them, less PAD_HALF_W, bound the distance to the fingers."""
    origin = _np(base.scene.env_origins)[0]
    out = []
    for arm in ARMS:
        robot = base.scene[arm]
        names, joints = list(robot.body_names), list(robot.joint_names)
        g = _np(robot.data.body_link_pose_w)[0, names.index("gripper")]
        q = _np(robot.data.joint_pos)[0]
        x, y, z, w = g[3:7]
        xax = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
        zax = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        tcp = g[:3] - origin + zax * TCP_OFFSET
        out.append(tcp)
        for jn, side in (("joint7", -1.0), ("joint8", 1.0)):
            c = tcp + side * xax * (abs(q[joints.index(jn)]) + 0.002)
            out += [c, c + zax * 0.0065, c + zax * 0.013]
    return out


def _state(base, spec: CotterRig) -> CotterState:
    _, _, bend, speed = _bends(base, spec)
    legs = _LEGS[spec.name]
    pos = np.asarray(spec.pos, float)
    gap = 1.0
    eye = pos + np.array([-spec.root_x - 0.0025, 0.0, 0.0])
    pts = _tool_points(base)
    for pt in pts:
        gap = min(gap, float(np.linalg.norm(pt - eye)) - 0.002 - PAD_HALF_W)
        for leg in LEGS:
            a = pos[:2] + (spec.root_x, 0.0)
            b = pos[:2] + leg_tip(spec, leg, bend[leg])
            d2 = point_segment_distance(pt[:2], a, b)
            gap = min(gap, math.hypot(d2, max(0.0, abs(pt[2] - pos[2]) - spec.leg_w / 2)) - PAD_HALF_W)
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 5 == 0:
            print(f"[cotter] {_DEBUG[0]:5d} bend long {math.degrees(bend['long']):6.1f} short "
                  f"{math.degrees(bend['short']):6.1f}  set {math.degrees(legs['long'].set):6.1f} "
                  f"{math.degrees(legs['short'].set):6.1f}  M {legs['long'].peak_moment:.3f} "
                  f"{legs['short'].peak_moment:.3f}  gap {gap * 100:.1f} cm  R-tcp(rig, mm) "
                  f"{np.round((pts[7] - pos) * 1000, 1)}", flush=True)
    half = np.array([0.03, 0.03, 0.03])
    return CotterState(
        pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
        low=pos - half, high=pos + half,
        bend_deg={k: math.degrees(v) for k, v in bend.items()},
        set_deg={k: math.degrees(legs[k].set) for k in LEGS},
        peak_set_deg={k: math.degrees(legs[k].peak) for k in LEGS},
        rebent={k: legs[k].rebent for k in LEGS},
        leg_speed=math.degrees(max(speed.values())), gap_m=gap,
        peak_moment_nm={k: legs[k].peak_moment for k in LEGS},
    )


# -- hooking yam_arena ---------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_cotterkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def cotter_task_object(spec):
        if isinstance(spec, CotterRig):
            _SPECS[spec.name] = spec
            _LEGS.setdefault(spec.name, {leg: Leg(spec.params) for leg in LEGS})
            return _arena_object(spec)
        return task_object(spec)

    def cotter_object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment.unwrapped, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = cotter_task_object, cotter_object_states
    scene._cotterkit = True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_cotterkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _apply(base)
        return write(*a, **k)

    isc.write_data_to_sim = write_data_to_sim
    isc._cotterkit = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_cotterkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def cotter_ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def cotter_reset(self, *a, **k):
        for legs in _LEGS.values():
            for leg in legs.values():
                leg.reset()
        return reset(self, *a, **k)

    cls._ensure_sim, cls.reset = cotter_ensure_sim, cotter_reset
    cls._cotterkit = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patch yam_arena.scene (imported only after Isaac Sim starts) and the embodiment as they are imported."""

    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module, patch = spec.loader.exec_module, self.targets[fullname]

        def patched_exec(module):
            exec_module(module)
            patch(module)

        spec.loader.exec_module = patched_exec
        return spec


def install() -> None:
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
__all__ = ["CotterRig", "CotterState", "Leg", "LegParams", "leg_tip", "point_segment_distance"]
