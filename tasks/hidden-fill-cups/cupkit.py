"""cupkit: sealed thin-walled cups whose side wall gives under a grip and buckles past a set force, for a yam_arena
task file.

yam_arena task files only take rigid primitives. This module adds one articulated object, :class:`Cup`, and hooks
yam_arena (without modifying it):

- ``yam_arena.scene.task_object`` builds a Cup as a FLOATING-base articulation. The root body is the cup's base disc
  and its recessed seal; it carries the fill mass. Its side wall is ``N_PANELS`` thin panels, each on a radial
  prismatic joint with a stiff spring (an implicit joint drive). A gripper squeezing the wall pushes the panels in
  against their springs, so the wall visibly gives and the fingers close a little further.
- Before every PHYSICS step (240 Hz, a wrapper on ``InteractiveScene.write_data_to_sim``) a :class:`CupMonitor` reads
  each panel's deflection. A panel whose spring force (stiffness x deflection) stays above the cup's buckling force
  for ``crush_hold_steps`` steps buckles: its spring's rest position moves 16 mm inward, so the wall stays dented and
  the fingers close through it. Robot contact with the root body (the base or the seal) is read from PhysX contact
  reports at the same rate.
- ``yam_arena.scene.object_states`` reports a :class:`CupState` per cup after every control step: the base pose, the
  panel deflections, the tool points of both grippers, and the monitor's latched violations.
- ``yam_arena.scripted.YamScriptedPolicy``: if the task module defines ``demo_controller``, ``yam_scripted`` runs it
  closed loop on the observation (camera images and joint positions), instead of a waypoint plan.
- Each embodiment reset clears the latches and the buckled panels.

The top half (``CupParams``, ``CupMonitor``) is plain NumPy, and it is what the tests exercise. Everything that needs
Isaac Sim is imported lazily.
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

N_PANELS = 8
PANEL = "panel_{}"
ARMS = ("left_robot", "right_robot")
TCP_OFFSET = 0.134  # tool point on the gripper link's z axis (yam_arena.constants.TCP_OFFSET)


# -- the cup's mechanics and the rules (pure Python) ---------------------------------------------------------------


@dataclass(frozen=True)
class CupParams:
    """Every number of a cup (m, kg, N)."""

    radius: float = 0.035  # outer wall radius
    height: float = 0.09  # base to rim
    wall: float = 0.003  # panel thickness
    base_h: float = 0.004  # base disc thickness; the panels start above it
    seal_top: float = 0.084  # top of the recessed seal, 6 mm below the rim
    seal_h: float = 0.003
    body_mass: float = 0.010  # base and seal, without the fill
    panel_mass: float = 0.002
    wall_k: float = 200.0  # N/m: one panel's spring. A flat pad on a panel also bears on its two neighbours'
    #                        edges, so the wall under a pad is about 5 x this stiff (1,000 N/m)
    wall_damping: float = 3.0  # N s/m
    armature: float = 0.005  # kg
    crush_empty_n: float = 1.5  # buckling force of an empty cup ...
    crush_per_kg: float = 34.0  # ... rising with the fill, which braces the wall from inside
    crush_hold_steps: int = 3  # physics steps in a row above it (12.5 ms at 240 Hz)
    dent_m: float = 0.016  # a buckled panel's new rest position, inward
    travel_m: float = 0.018  # joint limit, inward
    friction: float = 0.5  # wall, base, seal (PhysX "min" combine)
    grip_mu: float = 0.33  # effective pad-on-wall friction this gives in the simulation (measured: lift-off grips)
    fill_density: float = 1600.0  # kg/m^3, damp sand: sets the fill's level and the centre of mass

    def crush_n(self, fill: float) -> float:
        return self.crush_empty_n + self.crush_per_kg * fill

    def fill_level(self, fill: float) -> float:
        inner = self.radius - self.wall
        return min(fill / (self.fill_density * math.pi * inner**2), self.seal_top - self.seal_h - self.base_h)


@dataclass(frozen=True)
class Rules:
    """The scoring geometry and thresholds (m, deg, N). ``tray_inner``/``tray_outer``: (x0, x1, y0, y1)."""

    table_z: float
    tray_z: float  # tray floor's top surface
    tray_inner: tuple[float, float, float, float]
    tray_outer: tuple[float, float, float, float]
    airborne_m: float = 0.010  # base more than this above what is under it: off its support
    lifted_m: float = 0.020  # a lift counts from here
    slip_m: float = 0.010  # cup moved this far along the gripper while held
    grip_n: float = 0.3  # wall force on two opposite sides at least this: a wall grip
    grip_reach_m: float = 0.05  # gripper tool point this close to the cup's axis (horizontally) ...
    grip_band_m: float = 0.010  # ... and between this far above the base and this far below the rim
    tip_deg: float = 30.0  # ever tilted this far: knocked over
    upright_deg: float = 10.0
    fell_m: float = 0.05  # this far below the table top: off the table
    on_tray_tol_m: float = 0.003  # the base may overhang the tray's inner edge by this much
    touch_n: float = 0.3  # robot contact force on the base or seal ...
    touch_steps: int = 3  # ... for this many physics steps in a row


def tilt_deg(quat_xyzw) -> float:
    x, y, _, _ = quat_xyzw
    return math.degrees(math.acos(min(1.0, max(-1.0, 1 - 2 * (x * x + y * y)))))


def side_forces(deflection, p: CupParams) -> np.ndarray:
    """The force (N) a flat pad pressing on panel i exerts on the wall, from the panels' spring deflections (m,
    inward): panel i's own spring plus its two neighbours'. A neighbour slides along its own axis, 45 degrees off the
    pad's normal, so a neighbour deflected by d carries k d / sin45 of the pad's normal force. A pad pressing the
    wall in by x deflects panel i by x and its neighbours by x / sin45: 5 k x in all."""
    d = np.maximum(np.asarray(deflection, float), 0.0)
    return p.wall_k * (d + (np.roll(d, 1) + np.roll(d, -1)) / math.sin(math.pi / 4))


def _inside(xy, rect, margin: float) -> bool:
    x0, x1, y0, y1 = rect
    return x0 + margin <= xy[0] <= x1 - margin and y0 + margin <= xy[1] <= y1 - margin


class CupMonitor:
    """One cup's latches. :meth:`physics` runs every physics step on the panel deflections (m, positive inward) and
    the robot's contact force on the base or seal; it returns which panels have buckled. :meth:`control` runs after
    every control step on the cup's pose and the grippers' tool points."""

    def __init__(self, fill: float, params: CupParams, rules: Rules):
        self.fill, self.p, self.r = fill, params, rules
        self.reset()

    def reset(self) -> None:
        self.over = np.zeros(N_PANELS, int)
        self.buckled = np.zeros(N_PANELS, bool)
        self.peak_n = 0.0  # highest panel spring force so far
        self.touching = 0
        self.touched = False  # robot contact on the base or seal
        self.dropped = False  # off its support without a wall grip
        self.slipped = False  # moved more than slip_m along the gripper while held
        self.tipped = False
        self.fell = False
        self.doubled = False  # one gripper held it together with another cup
        self.lifted = False  # raised lifted_m in a wall grip at some point
        self.placed = False  # the last wall grip ended with the cup on the tray
        self.holder: str | None = None
        self.grip_offset = 0.0
        self.airborne = False

    @property
    def collapsed(self) -> bool:
        return bool(self.buckled.any())

    @property
    def failed(self) -> bool:
        return self.collapsed or self.touched or self.dropped or self.slipped or self.tipped or self.fell or self.doubled

    def physics(self, deflection, touch_force: float) -> np.ndarray:
        force = side_forces(deflection, self.p)
        self.peak_n = max(self.peak_n, float(force.max()))
        over = force > self.p.crush_n(self.fill)
        self.over = np.where(over, self.over + 1, 0)
        hit = self.over >= self.p.crush_hold_steps  # the wall buckles under that pad: its panel and both neighbours
        self.buckled |= hit | np.roll(hit, 1) | np.roll(hit, -1)
        self.touching = self.touching + 1 if touch_force > self.r.touch_n else 0
        self.touched |= self.touching >= self.r.touch_steps
        return self.buckled.copy()

    def support_z(self, xy) -> float:
        return self.r.tray_z if _inside(xy, self.r.tray_outer, 0.0) else self.r.table_z

    def on_tray(self, pos) -> bool:
        return (_inside(pos[:2], self.r.tray_inner, self.p.radius - self.r.on_tray_tol_m)
                and pos[2] - self.r.tray_z < self.r.airborne_m)

    def wall_grip(self, pos, deflection, tcps: dict) -> str | None:
        """The gripper holding the cup by its wall, or None: two opposite panels pressed in, and that gripper's tool
        point near the axis and level with the wall."""
        f = side_forces(deflection, self.p)
        half = N_PANELS // 2
        if not any(f[i] >= self.r.grip_n and f[i + half] >= self.r.grip_n for i in range(half)):
            return None
        best, best_d = None, self.r.grip_reach_m
        for arm, tcp in tcps.items():
            dist = math.hypot(tcp[0] - pos[0], tcp[1] - pos[1])
            up = tcp[2] - pos[2]
            if dist <= best_d and self.r.grip_band_m <= up <= self.p.height - self.r.grip_band_m:
                best, best_d = arm, dist
        return best

    def control(self, pos, quat_xyzw, deflection, tcps: dict) -> None:
        pos = np.asarray(pos, float)
        tilt = tilt_deg(quat_xyzw)
        self.tipped |= tilt > self.r.tip_deg
        self.fell |= pos[2] < self.r.table_z - self.r.fell_m
        height = pos[2] - self.support_z(pos[:2])
        holder = self.wall_grip(pos, deflection, tcps)
        if height > self.r.airborne_m and holder is None:
            self.dropped = True
        if holder is not None:
            offset = tcps[holder][2] - pos[2]
            if holder != self.holder:
                self.grip_offset = offset
            self.slipped |= abs(offset - self.grip_offset) > self.r.slip_m
            self.lifted |= pos[2] - self.r.table_z >= self.r.lifted_m
        elif self.holder is not None:
            self.placed = self.on_tray(pos)
        self.holder = holder
        self.airborne = height > self.r.airborne_m


def check_pairs(monitors: list[CupMonitor]) -> None:
    """One gripper carrying two cups off their supports at once: both fail."""
    for i, a in enumerate(monitors):
        for b in monitors[i + 1:]:
            if a.holder is not None and a.holder == b.holder and a.airborne and b.airborne:
                a.doubled = b.doubled = True


# -- the task-file object ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Cup(Box):
    """A sealed cup standing on the table: ``pos`` is its base centre on the table top; ``fill`` the hidden mass inside
    (kg); ``mass`` must be the total (``body_mass + N_PANELS * panel_mass + fill``)."""

    fill: float = 0.1
    params: CupParams = CupParams()
    wall_color: tuple[float, float, float] = (0.93, 0.93, 0.90)
    seal_color: tuple[float, float, float] = (0.85, 0.12, 0.10)

    def _check(self) -> None:
        super()._check()
        p = self.params
        if not 0.0 <= self.fill <= 0.6:
            raise ValueError(f"{self.name}: fill must be 0 to 0.6 kg")
        if self.mass is None or abs(self.mass - total_mass(self.fill, p)) > 1e-9:
            raise ValueError(f"{self.name}: mass must be total_mass(fill)")


def total_mass(fill: float, p: CupParams = CupParams()) -> float:
    return p.body_mass + N_PANELS * p.panel_mass + fill


def panel_frame(i: int, p: CupParams) -> tuple[tuple[float, float, float], float]:
    """Panel ``i``'s centre in the cup frame and its yaw (rad): panel 0 faces +x, numbered counter-clockwise."""
    a = 2 * math.pi * i / N_PANELS
    r = p.radius - p.wall / 2
    return (r * math.cos(a), r * math.sin(a), (p.base_h + p.height) / 2), a


@dataclass(frozen=True)
class CupState(ObjectState):
    """The base centre's pose (``pos`` is on the cup's bottom), the panel deflections (m, inward), the grippers' tool
    points and the latches."""

    deflection: np.ndarray = field(default_factory=lambda: np.zeros(N_PANELS))
    tcps: dict = field(default_factory=dict)
    fill: float = 0.0
    collapsed: bool = False
    touched: bool = False
    dropped: bool = False
    slipped: bool = False
    tipped: bool = False
    fell: bool = False
    doubled: bool = False
    lifted: bool = False
    placed: bool = False
    holder: str | None = None
    peak_n: float = 0.0
    on_tray: bool = False

    @property
    def failed(self) -> bool:
        return self.collapsed or self.touched or self.dropped or self.slipped or self.tipped or self.fell or self.doubled

    def tilt_deg(self) -> float:
        return tilt_deg(self.quat_xyzw)


# -- Isaac Sim side ------------------------------------------------------------------------------------------------

_SPECS: dict[str, Cup] = {}
_MON: dict[str, CupMonitor] = {}
_RULES: list[Rules] = []
_RUN: dict = {}  # runtime views, built on the first physics step
_DEBUG = bool(os.environ.get("CUPS_DEBUG"))


def configure(cups: list[Cup], rules: Rules) -> None:
    """Called by the task file: the cups and the scoring geometry."""
    _RULES[:] = [rules]
    for c in cups:
        _SPECS[c.name] = c
        _MON[c.name] = CupMonitor(c.fill, c.params, rules)


def monitors() -> dict[str, CupMonitor]:
    return _MON


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.cup_name]
    p = spec.params
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    mat = f"{prim_path}/Looks/paper"
    mcfg = sim_utils.RigidBodyMaterialCfg(static_friction=p.friction, dynamic_friction=p.friction,
                                          friction_combine_mode="min")
    mcfg.func(mat, mcfg)
    looks = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.6)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def collider(geom, color) -> None:
        prim = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(tuple(color)), stage=stage)

    def body(name, pos, yaw, mass) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(1.0)
        return path

    # Root: base disc and seal, carrying the fill. Centre of mass at half the fill's level.
    core = body("core", (0.0, 0.0, 0.0), 0.0, p.body_mass + spec.fill)
    mass_api = UsdPhysics.MassAPI(stage.GetPrimAtPath(core))
    level = p.fill_level(spec.fill)
    com_z = (p.body_mass * p.seal_top / 2 + spec.fill * (p.base_h + level / 2)) / (p.body_mass + spec.fill)
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, com_z))
    m, r_in = p.body_mass + spec.fill, p.radius - p.wall
    ixx = m * (3 * r_in**2 + max(level, 0.02) ** 2) / 12
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(ixx, ixx, m * r_in**2 / 2))
    PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(core)).CreateThresholdAttr(0.0)
    # The seal's collider is a 4 cm boss in its middle, so fingers pressing the wall in (up to 16 mm per side before
    # it buckles) never reach it; the full 6.2 cm red disc above it is visual only.
    for name, radius, h, zc, color, solid in (
        ("base", r_in, p.base_h, p.base_h / 2, spec.wall_color, True),
        ("seal", 0.020, p.seal_h, p.seal_top - p.seal_h / 2, spec.seal_color, True),
        ("seal_look", r_in - 0.001, 0.001, p.seal_top - 0.0005, spec.seal_color, False),
    ):
        cyl = UsdGeom.Cylinder.Define(stage, f"{core}/{name}")
        cyl.CreateAxisAttr("Z")
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(h)
        cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, zc))
        if solid:
            collider(cyl, color)
        else:
            bind_visual_material(str(cyl.GetPrim().GetPath()), look(tuple(color)), stage=stage)

    width = 2 * p.radius * math.tan(math.pi / N_PANELS)  # the octagon's side at the outer radius
    for i in range(N_PANELS):
        centre, yaw = panel_frame(i, p)
        path = body(PANEL.format(i), centre, yaw, p.panel_mass)
        cube = UsdGeom.Cube.Define(stage, f"{path}/wall")
        cube.CreateSizeAttr(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(p.wall, width, p.height - p.base_h))
        collider(cube, spec.wall_color)
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/{PANEL.format(i)}_joint")
        j.CreateAxisAttr("X")  # radial, outward
        j.CreateBody0Rel().SetTargets([Sdf.Path(core)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*centre))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(-p.travel_m)
        j.CreateUpperLimitAttr(0.0)
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")  # gains come from the actuator config
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(0.0)

    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(core))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(core))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Cup):
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
    class CupSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        cup_name: str = ""

    class CupObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"wall": ImplicitActuatorCfg(joint_names_expr=["panel_.*"], stiffness=p.wall_k,
                                                     damping=p.wall_damping, armature=p.armature)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return CupObject(
        name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=CupSpawnerCfg(cup_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)  # Isaac Lab 3.0 data are warp arrays with a .torch view
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)  # a bare warp array


class _Runtime:
    """Joint indices, the contact view (each cup's root body against every robot link) and the gripper bodies."""

    def __init__(self, base):
        from isaaclab_physx.physics import PhysxManager

        self.base = base
        self.names = list(_SPECS)
        self.joint_ids = {}
        roots = []
        for n in self.names:
            asset = base.scene[n]
            names = list(asset.joint_names)
            self.joint_ids[n] = [names.index(f"{PANEL.format(i)}_joint") for i in range(N_PANELS)]
            roots.append(_core_path(asset))
        links = []
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            links += list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
        view = PhysxManager.get_physics_sim_view()
        self.contact = view.create_rigid_contact_view(roots, filter_patterns=[links] * len(roots),
                                                      max_contact_data_count=64 * len(roots) * len(links))
        self.grip_body = {arm: base.scene[arm].find_bodies("gripper")[0][0] for arm in ARMS}
        self.origin = _np(base.scene.env_origins)[0]
        self.dt = float(base.physics_dt)
        self.steps = 0
        self.dirty = True  # write the panels' spring rest positions on the next step
        if _DEBUG:
            a = base.scene[self.names[0]]
            for attr in ("joint_pos_limits", "joint_stiffness", "joint_damping", "joint_armature", "default_joint_pos"):
                v = getattr(a.data, attr, None)
                print(f"[cups] {self.names[0]} {attr}: {None if v is None else np.round(_np(v)[0], 4)}", flush=True)
            print(f"[cups] joints {list(a.joint_names)} bodies {list(a.body_names)}", flush=True)

    def deflection(self, name: str) -> np.ndarray:
        q = _np(self.base.scene[name].data.joint_pos)[0]
        return -q[self.joint_ids[name]]

    def tcps(self) -> dict:
        out = {}
        for arm in ARMS:
            pose = _np(self.base.scene[arm].data.body_link_pose_w)[0, self.grip_body[arm]]
            out[arm] = pose[:3] - self.origin + _quat_z(pose[3:7]) * TCP_OFFSET
        return out

    def write_targets(self, name: str, buckled: np.ndarray) -> None:
        import torch

        asset = self.base.scene[name]
        target = torch.zeros((1, asset.num_joints), dtype=torch.float32, device=self.base.device)
        for i, b in enumerate(buckled):
            if b:
                target[0, self.joint_ids[name][i]] = -_SPECS[name].params.dent_m
        asset.set_joint_position_target_index(target=target)


def _core_path(asset) -> str:
    paths = asset.root_physx_view.link_paths
    paths = list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
    return next(p for p in paths if p.endswith("/core"))


def _quat_z(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def _physics(base) -> None:
    rt = _RUN.get("rt")
    if rt is None or rt.base is not base:
        rt = _RUN["rt"] = _Runtime(base)
    rt.steps += 1
    m = _np(rt.contact.get_contact_force_matrix(dt=rt.dt)).reshape(len(rt.names), -1, 3)
    touch = np.linalg.norm(m.sum(axis=1), axis=1)
    for k, n in enumerate(rt.names):
        mon = _MON[n]
        before = mon.buckled.copy()
        buckled = mon.physics(rt.deflection(n), float(touch[k]))
        if rt.dirty or (buckled != before).any():
            rt.write_targets(n, buckled)
    rt.dirty = False


def _state(base, name: str) -> CupState:
    rt = _RUN.get("rt")
    if rt is None or rt.base is not base:
        rt = _RUN["rt"] = _Runtime(base)
    asset = base.scene[name]
    mon = _MON[name]
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    pos = pose[:3] - rt.origin
    d, tcps = rt.deflection(name), rt.tcps()
    mon.control(pos, pose[3:7], d, tcps)
    r = mon.p.radius
    s = CupState(
        pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pos - (r, r, 0),
        high=pos + (r, r, mon.p.height), deflection=d, tcps=tcps, fill=mon.fill, collapsed=mon.collapsed,
        touched=mon.touched, dropped=mon.dropped, slipped=mon.slipped, tipped=mon.tipped, fell=mon.fell,
        doubled=mon.doubled, lifted=mon.lifted, placed=mon.placed, holder=mon.holder, peak_n=mon.peak_n,
        on_tray=mon.on_tray(pos),
    )  # fmt: skip
    if _DEBUG and rt.steps % 48 < 2 and name == rt.names[0]:
        print(f"[cups] t {rt.steps / 240:5.1f}", flush=True)
    if _DEBUG and rt.steps % 48 < 2:
        print(f"[cups]   {name} fill {mon.fill * 1000:3.0f} g pos {np.round(pos, 3)} tilt {s.tilt_deg():4.1f} "
              f"defl_mm {np.round(d * 1000, 1)} peak {mon.peak_n:4.2f}/{mon.p.crush_n(mon.fill):4.2f} N "
              f"tcpL {np.round(tcps['left_robot'], 3)} holder {mon.holder} lifted {mon.lifted} placed {mon.placed} collapsed {mon.collapsed} "
              f"touched {mon.touched} dropped {mon.dropped} slipped {mon.slipped} tipped {mon.tipped}", flush=True)
    return s


# -- hooking yam_arena ---------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_cupkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def cup_task_object(spec):
        if isinstance(spec, Cup):
            return _arena_object(spec)
        return task_object(spec)

    def cup_object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment.unwrapped, n) for n in names if n in _SPECS})
        check_pairs([_MON[n] for n in names if n in _MON])
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._cupkit = cup_task_object, cup_object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_cupkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _physics(base)
        return write(*a, **k)

    isc.write_data_to_sim, isc._cupkit = write_data_to_sim, True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_cupkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def cup_ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def cup_reset(self, *a, **k):
        for mon in _MON.values():
            mon.reset()
        if "rt" in _RUN:
            _RUN["rt"].dirty = True
        out = reset(self, *a, **k)
        for mon in _MON.values():  # the settle after env.reset() must not count
            mon.reset()
        return out

    cls._ensure_sim, cls.reset, cls._cupkit = cup_ensure_sim, cup_reset, True


def _patch_scripted(scripted) -> None:
    """``yam_scripted`` runs the task module's ``demo_controller`` (if any) closed loop on each observation."""
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_cupkit", False):
        return
    reset, act = cls.reset, cls.act

    def _controller(self):
        task = getattr(self, "_task", None)
        return getattr(task.module, "demo_controller", None) if task is not None else None

    def cup_reset(self, scene):
        make = _controller(self)
        if make is None:
            return reset(self, scene)
        self._cup_ctl = make(dict(left=self._bases["left"], right=self._bases["right"]), self._dt)
        return None

    def cup_act(self, observation):
        ctl = getattr(self, "_cup_ctl", None)
        if ctl is None or _controller(self) is None:
            return act(self, observation)
        from inspect_robots.types import Action, ActionChunk

        data, done = ctl.act(observation)
        meta = {"request_stop": True} if done else {}
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._cupkit = cup_reset, cup_act, True


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
