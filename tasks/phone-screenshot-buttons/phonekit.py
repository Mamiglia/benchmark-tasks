"""phonekit: a phone with three sprung edge buttons (side, volume up, volume down) standing in a loose desk stand,
and the phone's button logic, for a yam_arena task file.

yam_arena task files only take rigid primitives. This module adds two objects and hooks yam_arena (without modifying
it), the same way as our other kits:

- :class:`Phone` is a FLOATING-base articulation: the phone's body is the root, each button a small body on a
  prismatic joint normal to its edge, pushed outward against its stop by a preloaded spring (an implicit joint drive
  aimed past the stop), so it takes about 3 N to start moving it and it travels 0.35 mm.
- :class:`Stand` is one loose rigid body made of boxes: base plate, pedestal, and a cradle (seat, front lip and back
  rest) tilted back 20 degrees.
- Before every PHYSICS step (240 Hz) the :class:`Device` reads the three button depths and runs the phone's button
  logic (screenshot, volume, lock, power-off slider), and the robot's contact force on the phone is read from PhysX.
- ``yam_arena.scene.object_states`` reports a :class:`PhoneState` after every control step: pose, button depths,
  the device's event counts and the latched violations.
- ``yam_scripted`` runs the task module's ``demo_controller`` closed loop, as in our other kits.

The top half (``PhoneParams``, ``Device``, ``Seat``, ``grade``) is plain Python and is what the tests exercise.
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

ARMS = ("left_robot", "right_robot")
BUTTONS = ("side", "vol_up", "vol_down")
JOINT = "{}_joint"

# -- the phone (pure Python) -----------------------------------------------------------------------------------------


@dataclass(frozen=True)
class PhoneParams:
    """iPhone 12 (Apple's dimensional drawing, 2020-10-16). Phone frame: x through the back (screen faces -x), y to
    the phone's left edge as you look at the screen (the volume edge), z to the top. Metres, kg, N."""

    thick: float = 0.00739
    width: float = 0.07152
    length: float = 0.14671
    mass: float = 0.164
    # Button centres measured from the phone's top edge, and lengths (along z); the volume buttons are on the +y edge,
    # the side button on the -y edge. Face width (along x) 2.76 mm, centred on the edge.
    from_top: tuple = (0.04631, 0.03931, 0.05331)  # side, vol_up, vol_down
    lengths: tuple = (0.0178, 0.0113, 0.0113)
    face: float = 0.00276
    proud: float = 0.0006  # how far a button stands out of the frame (estimate; check with calipers)
    travel: float = 0.00035  # to its inner stop
    trigger: float = 0.00025  # the switch closes at this depth
    preload_n: float = 3.0  # force to start moving a button (estimate; measure as the README says)
    k: float = 1500.0  # N/m spring past the preload
    damping: float = 2.0
    armature: float = 0.002
    button_mass: float = 0.001
    button_depth: float = 0.002  # collider thickness along y; 0.6 mm of it outside the frame
    friction: float = 0.4  # aluminium frame on the stand's silicone seat and on the gripper pads

    def button_z(self, i: int) -> float:
        return self.length / 2 - self.from_top[i]

    def button_side(self, i: int) -> float:
        """+1 for the +y (volume) edge, -1 for the -y (side button) edge."""
        return -1.0 if BUTTONS[i] == "side" else 1.0

    def preload_m(self) -> float:
        return self.preload_n / self.k


@dataclass(frozen=True)
class DeviceRules:
    """The phone's button logic (iOS on a Face ID iPhone, from Apple's user guide; timings are estimates, see README).

    - Side button and volume up pressed together (second within ``chord_s`` of the first), and released within
      ``shot_s`` of the second press: a screenshot. Held together ``slider_s``: the power-off / Emergency SOS slider
      (also for side button and volume down).
    - A press that is not part of a pair: volume up or down (``chord_s`` after the press, if no pair formed),
      or for the side button, lock (or Siri, if held), counted when it is released."""

    chord_s: float = 0.20
    shot_s: float = 1.0
    slider_s: float = 2.0


EVENTS = ("screenshot", "volume_up", "volume_down", "lock", "slider")


class Device:
    """Runs every physics step on which buttons are in past their trigger depth; counts the events."""

    def __init__(self, rules: DeviceRules = DeviceRules()):
        self.r = rules
        self.reset()

    def reset(self) -> None:
        self.t = 0.0
        self.down = dict.fromkeys(BUTTONS, False)
        self.t_down = dict.fromkeys(BUTTONS, 0.0)
        self.used = dict.fromkeys(BUTTONS, False)  # this press is part of a pair (or already counted)
        self.pair: tuple[str, str] | None = None
        self.pair_t = 0.0
        self.pair_done = False
        self.events = dict.fromkeys(EVENTS, 0)
        self.log: list[tuple[float, str]] = []

    def _event(self, name: str) -> None:
        self.events[name] += 1
        self.log.append((round(self.t, 4), name))

    def step(self, t: float, pressed: dict) -> None:
        self.t = t
        r = self.r
        for b in BUTTONS:
            now = bool(pressed.get(b, False))
            if now and not self.down[b]:
                self.t_down[b], self.used[b] = t, False
            if not now and self.down[b]:
                self._release(b)
            self.down[b] = now
        # A new pair: the side button with a volume button, both down, pressed within chord_s of each other.
        if self.pair is None:
            for vol in ("vol_up", "vol_down"):
                if (self.down["side"] and self.down[vol] and not self.used["side"] and not self.used[vol]
                        and abs(self.t_down["side"] - self.t_down[vol]) <= r.chord_s):
                    self.pair, self.pair_t, self.pair_done = ("side", vol), max(self.t_down["side"], self.t_down[vol]), False
                    self.used["side"] = self.used[vol] = True
                    break
        if self.pair is not None and not self.pair_done and all(self.down[b] for b in self.pair):
            if t - self.pair_t >= r.slider_s:
                self._event("slider")
                self.pair_done = True
        # A volume press that no pair claimed within chord_s changes the volume.
        for vol, ev in (("vol_up", "volume_up"), ("vol_down", "volume_down")):
            if self.down[vol] and not self.used[vol] and t - self.t_down[vol] > r.chord_s:
                self.used[vol] = True
                self._event(ev)

    def _release(self, b: str) -> None:
        if self.pair is not None and b in self.pair:
            if not self.pair_done:
                if self.pair[1] == "vol_up" and self.t - self.pair_t <= self.r.shot_s:
                    self._event("screenshot")
                self.pair_done = True
            if not any(self.down[x] for x in self.pair if x != b):
                self.pair = None
            return
        if b == "side" and not self.used["side"]:
            self.used["side"] = True
            self._event("lock")
        if b != "side" and not self.used[b]:  # a volume press shorter than chord_s
            self.used[b] = True
            self._event("volume_up" if b == "vol_up" else "volume_down")


@dataclass(frozen=True)
class SeatRules:
    """The phone stays in its stand: none of these, at any control step, relative to its settled start pose."""

    lift_m: float = 0.010  # centre more than this up or down
    slide_m: float = 0.015  # centre more than this sideways (y) or forward/back (x)
    tilt_deg: float = 10.0  # turned more than this


def angle_deg(q0, q1) -> float:
    """Angle between two orientations (quaternions xyzw)."""
    d = abs(float(np.dot(np.asarray(q0, float), np.asarray(q1, float))))
    return math.degrees(2 * math.acos(min(1.0, d)))


class Seat:
    def __init__(self, rules: SeatRules = SeatRules()):
        self.r = rules
        self.reset()

    def reset(self) -> None:
        self.ref = None
        self.out = False
        self.worst = (0.0, 0.0, 0.0)

    def control(self, pos, quat_xyzw) -> None:
        pos = np.asarray(pos, float)
        if self.ref is None:
            self.ref = (pos.copy(), np.asarray(quat_xyzw, float).copy())
            return
        d = pos - self.ref[0]
        horiz = float(np.hypot(d[0], d[1]))
        tilt = angle_deg(self.ref[1], quat_xyzw)
        self.worst = tuple(max(a, b) for a, b in zip(self.worst, (abs(float(d[2])), horiz, tilt), strict=True))
        self.out |= abs(d[2]) > self.r.lift_m or horiz > self.r.slide_m or tilt > self.r.tilt_deg


def grade(events: dict, out: bool, touching: bool) -> float:
    """The score from the device's event counts, the seat latch and whether the robot still touches the phone."""
    if out:
        return 0.0
    if events.get("slider", 0):
        return 0.1
    shots = events.get("screenshot", 0)
    others = sum(events.get(e, 0) for e in ("volume_up", "volume_down", "lock"))
    if shots == 1 and others == 0 and not touching:
        return 1.0
    if shots >= 1:
        return 0.5
    if others:
        return 0.3
    return 0.0


# -- the task-file objects -----------------------------------------------------------------------------------------


def _pitch_quat(pitch: float) -> tuple[float, float, float, float]:
    """xyzw for a rotation of ``pitch`` about world y (positive leans +z toward +x)."""
    return (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))


@dataclass(frozen=True)
class Phone(Box):
    """The phone: ``pos`` is its centre, ``pitch`` its lean back from upright (rad, about y). ``size`` is unused."""

    params: PhoneParams = PhoneParams()
    pitch: float = 0.0
    frame_color: tuple[float, float, float] = (0.80, 0.84, 0.90)
    screen_color: tuple[float, float, float] = (0.04, 0.05, 0.07)


@dataclass(frozen=True)
class Stand(Box):
    """A desk phone stand: ``pos`` is its base plate's centre on the table. Its cradle leans back ``pitch``; ``seat``
    is the seat's back-bottom corner line (the phone's back-bottom edge) in the stand's frame, (x, z)."""

    pitch: float = math.radians(20)
    seat: tuple[float, float] = (0.012, 0.030)
    friction: float = 1.0


@dataclass(frozen=True)
class PhoneState(ObjectState):
    depth: dict = field(default_factory=dict)  # button depths (m)
    events: dict = field(default_factory=dict)
    out: bool = False
    touching: bool = False  # robot contact on the phone now
    worst: tuple = (0.0, 0.0, 0.0)


# -- Isaac Sim side ------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_DEV = Device()
_SEAT = Seat()
_RUN: dict = {}
_DEBUG = bool(os.environ.get("PHONE_DEBUG"))


def configure(phone: Phone, stand: Stand, rules: DeviceRules = DeviceRules(), seat: SeatRules = SeatRules()) -> None:
    _SPECS.clear()
    _SPECS[phone.name], _SPECS[stand.name] = phone, stand
    _DEV.r, _SEAT.r = rules, seat
    _DEV.reset()
    _SEAT.reset()


def device() -> Device:
    return _DEV


def seat() -> Seat:
    return _SEAT


def _usd_tools(prim_path, friction, color_list):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    mat = f"{prim_path}/Looks/mat"
    mcfg = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
    mcfg.func(mat, mcfg)
    looks = {}

    def look(color) -> str:
        color = tuple(color)
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def cube(path, size, pos, color, solid=True, pitch=0.0, offset=0.001):
        c = UsdGeom.Cube.Define(stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if pitch:
            c.AddOrientOp().Set(Gf.Quatf(math.cos(pitch / 2), 0, math.sin(pitch / 2), 0))
        c.AddScaleOp().Set(Gf.Vec3f(*size))
        prim = c.GetPrim()
        if solid:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(offset)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(prim.GetPath()), mat, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return prim

    return stage, cube


def _root_xform(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom

    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    return root


def _spawn_phone(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    spec: Phone = _SPECS[cfg.kit_name]
    p = spec.params
    from isaaclab.sim.utils.stage import get_current_stage

    root = _root_xform(get_current_stage(), prim_path, translation, orientation)
    stage, cube = _usd_tools(prim_path, p.friction, ())
    dbl = UsdGeom.XformOp.PrecisionDouble

    def body(name, pos, mass) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(0.5)
        return path

    core = body("core", (0.0, 0.0, 0.0), p.mass - 3 * p.button_mass)
    cube(f"{core}/frame", (p.thick, p.width, p.length), (0, 0, 0), spec.frame_color)
    # Screen (visual): black glass over the front face, 3.5 mm in from the edges; a grey notch.
    cube(f"{core}/screen", (0.0004, p.width - 0.007, p.length - 0.007), (-p.thick / 2 - 0.0002, 0, 0),
         spec.screen_color, solid=False)
    cube(f"{core}/notch", (0.0006, 0.035, 0.005), (-p.thick / 2 - 0.0003, 0, p.length / 2 - 0.006),
         (0.02, 0.02, 0.02), solid=False)
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(core)).CreateThresholdAttr(0.0)
    for i, name in enumerate(BUTTONS):
        s = p.button_side(i)
        # Button body centre: its outer face p.proud outside the frame.
        yc = s * (p.width / 2 + p.proud - p.button_depth / 2)
        path = body(name, (0.0, yc, p.button_z(i)), p.button_mass)
        color = (0.70, 0.74, 0.80) if name != "side" else (0.70, 0.74, 0.80)
        cube(f"{path}/cap", (p.face, p.button_depth, p.lengths[i]), (0, 0, 0), color, offset=0.0005)
        PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.0)
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/{JOINT.format(name)}")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([Sdf.Path(core)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        # Joint frame turned 180 degrees about z for the -y edge, so +q is always outward and -q pressed in.
        rot = Gf.Quatf(1, 0, 0, 0) if s > 0 else Gf.Quatf(0, 0, 0, 1)
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, yc, p.button_z(i)))
        j.CreateLocalRot0Attr(rot)
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(rot)
        j.CreateLowerLimitAttr(-p.travel)
        j.CreateUpperLimitAttr(0.0)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(core))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(core))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def stand_parts(st: Stand, p: PhoneParams) -> list:
    """The stand's boxes in its own frame: (name, size, centre, pitch). The cradle's pieces are given in the cradle
    frame (origin on the seat's back-bottom corner line, leaning back st.pitch) and turned into the stand frame."""
    sx, sz = st.seat
    c, s = math.cos(st.pitch), math.sin(st.pitch)

    def cradle(u, w):  # cradle (u across the phone's thickness, +u to the back; w up the phone) -> stand (x, z)
        return (sx + u * c + w * s, sz - u * s + w * c)

    parts = [("base", (0.090, 0.080, 0.005), (0.0, 0.0, 0.0025), 0.0)]
    # Pedestal: a block from the base plate up under the seat.
    top = sz - 0.006 * c
    parts.append(("pedestal", (0.030, 0.050, top - 0.005), (sx - 0.008, 0.0, 0.005 + (top - 0.005) / 2), 0.0))
    seat_d, lip_h, back_h, plate = 0.016, 0.013, 0.075, 0.005
    for name, size, (u, w) in (
        ("seat", (seat_d, 0.060, plate), (-seat_d / 2 + 0.002, -plate / 2)),
        ("lip", (plate, 0.060, lip_h), (-p.thick - plate / 2 - 0.0015, lip_h / 2 - plate)),
        ("back", (plate, 0.045, back_h), (plate / 2, back_h / 2 - plate)),
    ):
        x, z = cradle(u, w)
        parts.append((name, size, (x, 0.0, z), st.pitch))
    return parts


def phone_pose_in_stand(st: Stand, p: PhoneParams) -> tuple[float, float]:
    """The phone's centre (x, z) in the stand frame, resting on the seat against the back rest."""
    sx, sz = st.seat
    c, s = math.cos(st.pitch), math.sin(st.pitch)
    u, w = -p.thick / 2, p.length / 2 + 0.0005
    return sx + u * c + w * s, sz - u * s + w * c


def _spawn_stand(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import PhysxSchema, UsdPhysics

    spec: Stand = _SPECS[cfg.kit_name]
    phone = next(s for s in _SPECS.values() if isinstance(s, Phone))
    root = _root_xform(get_current_stage(), prim_path, translation, orientation)
    stage, cube = _usd_tools(prim_path, spec.friction, ())
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(0.5)
    for name, size, pos, pitch in stand_parts(spec, phone.params):
        cube(f"{prim_path}/{name}", size, pos, spec.color, pitch=pitch)
    return prim


def _arena_object(spec):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    is_phone = isinstance(spec, Phone)

    @configclass
    class KitSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_phone if is_phone else _spawn_stand)
        kit_name: str = ""

    if is_phone:
        p = spec.params

        class PhoneObject(Object):
            def _generate_articulation_cfg(self):
                actuators = {"buttons": ImplicitActuatorCfg(joint_names_expr=[".*_joint"], stiffness=p.k,
                                                            damping=p.damping, armature=p.armature)}
                cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
                return self._add_initial_pose_to_cfg(cfg)

        return PhoneObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                           spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                           initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_pitch_quat(spec.pitch)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _link_paths(asset) -> list:
    paths = asset.root_physx_view.link_paths
    return list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)


class _Runtime:
    def __init__(self, base):
        from isaaclab_physx.physics import PhysxManager

        self.base = base
        self.phone = next(n for n, s in _SPECS.items() if isinstance(s, Phone))
        asset = base.scene[self.phone]
        names = list(asset.joint_names)
        self.jid = [names.index(JOINT.format(b)) for b in BUTTONS]
        bodies = _link_paths(asset)
        links = []
        for arm in ARMS:
            links += _link_paths(base.scene[arm])
        view = PhysxManager.get_physics_sim_view()
        self.contact = view.create_rigid_contact_view(bodies, filter_patterns=[links] * len(bodies),
                                                      max_contact_data_count=64 * len(bodies) * len(links))
        self.origin = _np(base.scene.env_origins)[0]
        self.dt = float(base.physics_dt)
        self.p = _SPECS[self.phone].params
        self.steps = 0
        self.dirty = True
        self.touch = 0.0

    def depths(self) -> dict:
        q = _np(self.base.scene[self.phone].data.joint_pos)[0]
        return {b: float(-q[j]) for b, j in zip(BUTTONS, self.jid, strict=True)}

    def write_targets(self) -> None:
        import torch

        asset = self.base.scene[self.phone]
        target = torch.zeros((1, asset.num_joints), dtype=torch.float32, device=self.base.device)
        for j in self.jid:
            target[0, j] = self.p.preload_m()
        asset.set_joint_position_target_index(target=target)


def _rt(base) -> _Runtime:
    rt = _RUN.get("rt")
    if rt is None or rt.base is not base:
        rt = _RUN["rt"] = _Runtime(base)
    return rt


def _physics(base) -> None:
    rt = _rt(base)
    if rt.dirty:
        rt.write_targets()
        rt.dirty = False
    rt.steps += 1
    d = rt.depths()
    _DEV.step(rt.steps * rt.dt, {b: d[b] >= rt.p.trigger for b in BUTTONS})
    m = _np(rt.contact.get_contact_force_matrix(dt=rt.dt)).reshape(-1, 3)
    rt.touch = float(np.linalg.norm(m, axis=1).sum())
    if _DEBUG and rt.steps % 24 == 0:
        print(f"[phone] t {rt.steps * rt.dt:6.3f} depth_mm {[round(v * 1000, 3) for v in d.values()]} "
              f"touch {rt.touch:5.2f} events {_DEV.events}", flush=True)


def _state(base, name: str):
    rt = _rt(base)
    asset = base.scene[name]
    spec = _SPECS[name]
    if isinstance(spec, Phone):
        pose = _np(asset.data.root_link_pose_w)[0]
        vel = _np(asset.data.root_vel_w)[0]
    else:
        pose = _np(asset.data.root_link_pose_w)[0]
        vel = _np(asset.data.root_vel_w)[0]
    pos = pose[:3] - rt.origin
    if not isinstance(spec, Phone):
        return ObjectState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pos, high=pos)
    _SEAT.control(pos, pose[3:7])
    p = spec.params
    half = np.array([p.length, p.width, p.length]) / 2
    s = PhoneState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pos - half, high=pos + half,
                   depth=rt.depths(), events=dict(_DEV.events), out=_SEAT.out, touching=rt.touch > 0.05,
                   worst=_SEAT.worst)
    if _DEBUG:
        print(f"[phone] ctl pos {np.round(pos, 4)} worst {np.round(_SEAT.worst, 4)} out {_SEAT.out} "
              f"touch {rt.touch:.2f} log {_DEV.log}", flush=True)
    return s


# -- hooking yam_arena ---------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_phonekit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def kit_task_object(spec):
        if isinstance(spec, (Phone, Stand)):
            return _arena_object(spec)
        return task_object(spec)

    def kit_object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment.unwrapped, n) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._phonekit = kit_task_object, kit_object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_phonekit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _physics(base)
        return write(*a, **k)

    isc.write_data_to_sim, isc._phonekit = write_data_to_sim, True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_phonekit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def kit_ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def kit_reset(self, *a, **k):
        if "rt" in _RUN:
            _RUN["rt"].dirty = True
        out = reset(self, *a, **k)
        _DEV.reset()  # the settle after env.reset() must not count
        _SEAT.reset()
        if "rt" in _RUN:
            _RUN["rt"].steps = 0
        return out

    cls._ensure_sim, cls.reset, cls._phonekit = kit_ensure_sim, kit_reset, True


def _patch_scripted(scripted) -> None:
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_phonekit", False):
        return
    reset, act = cls.reset, cls.act

    def _controller(self):
        task = getattr(self, "_task", None)
        return getattr(task.module, "demo_controller", None) if task is not None else None

    def kit_reset(self, scene):
        make = _controller(self)
        if make is None:
            return reset(self, scene)
        self._kit_ctl = make(dict(left=self._bases["left"], right=self._bases["right"]), self._dt)
        return None

    def kit_act(self, observation):
        ctl = getattr(self, "_kit_ctl", None)
        if ctl is None or _controller(self) is None:
            return act(self, observation)
        from inspect_robots.types import Action, ActionChunk

        data, done = ctl.act(observation)
        meta = {"request_stop": True} if done else {}
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._phonekit = kit_reset, kit_act, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted}


class _Finder(importlib.abc.MetaPathFinder):
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
