"""cordkit: a cord tied to a fixed anchor that can be cut with a knife, for a yam_arena task file.

yam_arena task files only take single primitives. This module adds one object, :class:`CordRig`, spawned as a static
(``BASE``) Arena asset that holds several bodies:

- ``anchor``: a kinematic body (any boxes). The cord is tied to it at ``pos``.
- ``seg_00`` ... ``seg_NN``: the cord, a chain of capsules joined end to end by free spherical joints (outside any
  articulation). The first joint ties it to the anchor, the last one to ``tag``, a small block on its free end. The
  chain is spawned straight along ``lay`` (default: hanging straight down).
- ``knife`` (handle and bolster) and ``blade``: two rigid bodies joined by a fixed joint, so contacts on the blade
  are reported apart from the handle.

Cutting. PhysX cannot cut, so a joint of the cord is switched off when the blade's edge presses on the segment next to
it hard enough. Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit
reads PhysX's contact force between the blade body and each segment, from the step just simulated, and takes its
component against the blade's edge normal (the force that pushes back into the edge). When that component is at
least ``cut_n`` for ``cut_hold`` physics steps in a row on one segment, the joint at the end of that segment nearer
the edge is disabled. The force is a measured contact force: a slack cord moves away from the blade and the force
stays near the cord's weight; a cord held at both ends resists and the force rises. Forces on the flat of the blade
or on its spine do not count. The same contact readout latches any contact between the blade and a robot link.

``object_states`` reports a :class:`CordState` for the rig: body poses, the cuts (joint, point, distance to the other
gripper), the blade-touch latch and the arms' tool points. ``on_reset`` callbacks run after every embodiment reset.
Importing the module hooks ``yam_arena.scene``, ``yam_arena.inspect_embodiment`` and ``yam_arena.demo`` (a
``Waypoint.yaw`` may be a 3x3 rotation matrix). yam_arena itself is not modified. Pure Python at import; Isaac Sim is
imported lazily.
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

Color = tuple[float, float, float]
BoxSpec = tuple[tuple[float, float, float], tuple[float, float, float], Color]  # centre, size (m), sRGB colour
TCP_OFFSET = 0.134  # yam_arena.constants.TCP_OFFSET: tool point on the gripper link's z axis
ARMS = ("left_robot", "right_robot")


@dataclass(frozen=True)
class CordRig(Box):
    """Everything is in the env frame (x forward, y left, z up, m). ``pos`` is where the cord is tied to the anchor."""

    anchor_boxes: tuple[BoxSpec, ...] = ()  # kinematic, in the env frame
    lay: tuple[float, float, float] = (0.0, 0.0, -1.0)  # the direction the cord is spawned along, from ``pos``
    n_seg: int = 12
    seg_len: float = 0.01
    radius: float = 0.0015
    seg_mass: float = 0.002
    seg_inertia: float = 2e-7  # kg m^2 per axis (raised from a thin rod's value for solver stability)
    seg_colors: tuple[Color, ...] = ((0.95, 0.93, 0.85),)
    tag_size: tuple[float, float, float] = (0.01, 0.03, 0.03)
    tag_mass: float = 0.01
    tag_color: Color = (1.0, 0.8, 0.1)
    knife_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)  # handle centre; knife frame = env axes at the start
    handle_boxes: tuple[BoxSpec, ...] = ()  # knife frame
    knife_mass: float = 0.04
    blade_centre: tuple[float, float, float] = (0.0, 0.0, 0.0)  # knife frame
    blade_size: tuple[float, float, float] = (0.02, 0.08, 0.002)  # knife frame
    table_z: float | None = None  # table top (env frame): the blade's lowest corner within 1 mm of it latches
    edge_normal: tuple[float, float, float] = (1.0, 0.0, 0.0)  # knife frame: the way the edge faces
    blade_axis: tuple[float, float, float] = (0.0, 1.0, 0.0)  # knife frame: along the edge
    blade_mass: float = 0.02
    blade_color: Color = (0.78, 0.80, 0.83)
    friction: float = 1.0
    cut_n: float = 4.0  # N, edge-normal contact force that parts the cord ...
    cut_hold: int = 3  # ... held for this many physics steps in a row
    touch_n: float = 0.05  # N: any blade-robot contact force above this counts as a touch

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a CordRig is a fixed asset (mass None); its bodies carry their own")

    @property
    def dir(self) -> np.ndarray:
        return np.asarray(self.lay, float) / np.linalg.norm(self.lay)

    def seg_centre(self, k: int) -> np.ndarray:
        return np.asarray(self.pos) + self.dir * (k + 0.5) * self.seg_len

    @property
    def tag_centre(self) -> np.ndarray:
        return np.asarray(self.pos) + self.dir * (self.n_seg * self.seg_len + self.tag_size[2] / 2)

    @property
    def quat_wxyz(self) -> np.ndarray:
        """Spawn rotation of the segments and the tag: their local +z (toward the anchor) along -``lay``."""
        v = -self.dir
        if v[2] < -1 + 1e-9:
            raise ValueError(f"{self.name}: lay must not point straight up")
        q = np.r_[1.0 + v[2], np.cross((0.0, 0.0, 1.0), v)]
        return q / np.linalg.norm(q)

    @property
    def length(self) -> float:
        return self.n_seg * self.seg_len


@dataclass(frozen=True)
class Cut:
    joint: int  # 0 = at the anchor, n_seg = at the tag
    point: np.ndarray  # env frame
    hand_m: float  # distance from the cut to the tool point of the gripper not holding the knife
    t: float  # s since the reset
    held_m: float = 0.0  # distance from that tool point to the nearest cord segment or the tag, when the cord parted


@dataclass(frozen=True)
class CordState(ObjectState):
    """The rig after a control step. ``pos`` is the tag; every position is in the env frame."""

    seg_pos: np.ndarray = None
    tag_pos: np.ndarray = None
    knife_pos: np.ndarray = None  # handle centre
    knife_R: np.ndarray = None
    knife_vel: np.ndarray = None
    blade_pos: np.ndarray = None
    tcp: dict = field(default_factory=dict)  # arm -> tool point
    cuts: tuple = ()
    touched: str | None = None  # the robot link the blade first touched, if any
    blade_on_table: bool = False  # the blade touched the table at some physics step
    peak_edge_n: float = 0.0


def blade_low_z(blade_p, blade_R, size) -> float:
    """Height of the blade box's lowest corner."""
    return float(blade_p[2] - np.abs(np.asarray(blade_R)[2]) @ np.asarray(size, float) / 2)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


FORCE_SIGN = 1.0  # PhysX's force matrix is the force on the sensor body (the blade) from each filter body


def edge_force(f_on_blade, blade_R, normal=(1.0, 0.0, 0.0)) -> float:
    """Component of the contact force on the blade that pushes back into its edge (the edge faces ``normal``, knife
    frame). Forces on the flat or the spine give zero or less."""
    return -float(np.dot(f_on_blade, np.asarray(blade_R) @ np.asarray(normal, float)))


def nearer_joint(k: int, seg_p, seg_R, edge_p, edge_dir, seg_len: float) -> int:
    """Of segment k's two joints (k at its top, k+1 at its bottom), the one nearer the blade's edge line."""
    axis = np.asarray(seg_R)[:, 2] * seg_len / 2
    d = np.asarray(edge_dir, float) / np.linalg.norm(edge_dir)

    def dist(p):
        v = p - np.asarray(edge_p)
        return np.linalg.norm(v - d * np.dot(v, d))

    return k if dist(np.asarray(seg_p) + axis) <= dist(np.asarray(seg_p) - axis) else k + 1


class Cutter:
    """The cut rule, fed one physics step at a time (pure NumPy, so the tests can drive it)."""

    def __init__(self, spec: CordRig):
        self.spec = spec
        self.reset()

    def reset(self) -> None:
        self.run = np.zeros(self.spec.n_seg, int)
        self.cut = np.zeros(self.spec.n_seg + 1, bool)
        self.cuts: list[Cut] = []
        self.touched: str | None = None
        self.on_table = False
        self.peak = 0.0

    def step(self, edge_n, touch_n: dict[str, float]) -> list[int]:
        """``edge_n``: edge-normal force per segment (N). ``touch_n``: |force| per robot link. Returns the segments
        whose press has lasted ``cut_hold`` steps (the caller picks and disables the joint)."""
        edge_n = np.asarray(edge_n, float)
        self.peak = max(self.peak, float(edge_n.max(initial=0.0)))
        for link, f in touch_n.items():
            if f > self.spec.touch_n and self.touched is None:
                self.touched = link
        self.run = np.where(edge_n >= self.spec.cut_n, self.run + 1, 0)
        hit = [k for k in range(self.spec.n_seg) if self.run[k] >= self.spec.cut_hold]
        for k in hit:
            self.run[k] = 0
        return hit


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, CordRig] = {}
_PATHS: dict[str, str] = {}  # rig name -> its root prim (the first spawned: env 0)


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    s = _SPECS[cfg.rig_name]
    _PATHS.setdefault(s.name, prim_path)
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    material = f"{prim_path}/Looks/physics"
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=s.friction, dynamic_friction=s.friction)
    mat.func(material, mat)
    looks: dict = {}

    def look(color) -> str:
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(looks[color], vis)
        return looks[color]

    def collider(prim, offset=0.002) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(offset)
        col.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material, stage=stage)

    def boxes(path, specs, offset=0.002) -> None:
        for i, (centre, size, color) in enumerate(specs):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collider(cube.GetPrim(), offset)
            bind_visual_material(str(cube.GetPath()), look(tuple(color)), stage=stage)

    def body(name, pos, mass, inertia=None, kinematic=False, damping=(0.0, 0.05), q=(1.0, 0.0, 0.0, 0.0)) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*pos))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(*map(float, q)))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia, inertia, inertia))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(damping[0])
        rb.CreateAngularDampingAttr(damping[1])
        return path

    def joint(name, cls, b0, b1, p0, p1, rot0=(1.0, 0.0, 0.0, 0.0)) -> None:
        j = cls.Define(stage, f"{prim_path}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*p0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*p1))
        j.CreateLocalRot0Attr(Gf.Quatf(*map(float, rot0)))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        j.CreateJointEnabledAttr(True)

    eye, q = np.asarray(s.pos, float), s.quat_wxyz
    anchor = body("anchor", tuple(eye), 1.0, kinematic=True)
    boxes(anchor, [(tuple(np.subtract(c, eye)), sz, col) for c, sz, col in s.anchor_boxes], 0.003)

    half = s.seg_len / 2
    prev, prev_p, rot0 = anchor, (0.0, 0.0, 0.0), q
    for k in range(s.n_seg):
        seg = body(f"seg_{k:02d}", tuple(s.seg_centre(k)), s.seg_mass, s.seg_inertia, damping=(0.5, 1.0), q=q)
        cap = UsdGeom.Capsule.Define(stage, f"{seg}/capsule")
        cap.CreateRadiusAttr(s.radius)
        cap.CreateHeightAttr(s.seg_len - 2 * s.radius)
        cap.CreateAxisAttr("Z")
        collider(cap.GetPrim(), 0.002)
        bind_visual_material(str(cap.GetPath()), look(tuple(s.seg_colors[k % len(s.seg_colors)])), stage=stage)
        joint(f"cord_joint_{k:02d}", UsdPhysics.SphericalJoint, prev, seg, prev_p, (0.0, 0.0, half), rot0)
        prev, prev_p, rot0 = seg, (0.0, 0.0, -half), (1.0, 0.0, 0.0, 0.0)
    tag = body("tag", tuple(s.tag_centre), s.tag_mass, damping=(0.5, 1.0), q=q)
    boxes(tag, [((0.0, 0.0, 0.0), s.tag_size, s.tag_color)])
    joint(f"cord_joint_{s.n_seg:02d}", UsdPhysics.SphericalJoint, prev, tag, prev_p, (0.0, 0.0, s.tag_size[2] / 2))

    knife = body("knife", s.knife_pos, s.knife_mass)
    boxes(knife, s.handle_boxes, 0.003)
    blade = body("blade", tuple(np.add(s.knife_pos, s.blade_centre)), s.blade_mass)
    boxes(blade, [((0.0, 0.0, 0.0), s.blade_size, s.blade_color)], 0.002)
    report = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(blade))
    report.CreateThresholdAttr(0.0)
    joint("blade_fixed", UsdPhysics.FixedJoint, knife, blade, s.blade_centre, (0.0, 0.0, 0.0))
    return root.GetPrim()


def _arena_object(spec: CordRig):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class _Cfg(SpawnerCfg):
        func: object = clone(_spawn)
        rig_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=_Cfg(rig_name=spec.name),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# -- runtime ----------------------------------------------------------------------------------------------------


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


class _Runtime:
    """PhysX views and the cutter for one rig (env 0)."""

    def __init__(self, base, spec: CordRig):
        from isaaclab_physx.physics import PhysxManager
        from isaaclab.sim.utils.stage import get_current_stage

        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        self.stage = get_current_stage()
        self.names = [f"seg_{k:02d}" for k in range(spec.n_seg)] + ["tag", "knife", "blade"]
        view = PhysxManager.get_physics_sim_view()
        self.bodies = view.create_rigid_body_view([f"{root}/{n}" for n in self.names])
        self.links = []  # (arm, link name, prim path) of every robot rigid body
        for arm in ARMS:
            paths = base.scene[arm].root_physx_view.link_paths
            for p in (paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths):
                self.links.append((arm, p.rsplit("/", 1)[-1], p))
        filters = [f"{root}/seg_{k:02d}" for k in range(spec.n_seg)] + [p for _, _, p in self.links]
        self.contact = view.create_rigid_contact_view([f"{root}/blade"], filter_patterns=[filters],
                                                      max_contact_data_count=256)
        self.joints = [self.stage.GetPrimAtPath(f"{root}/cord_joint_{j:02d}") for j in range(spec.n_seg + 1)]
        self.cutter = Cutter(spec)
        self.dt = float(base.physics_dt)
        self.t = 0.0
        self.start = _arr(self.bodies.get_transforms()).copy()
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("CUT_DEBUG"))
        self.step_peak = self.step_neg = self.step_touch = 0.0

    def tcps(self) -> dict[str, np.ndarray]:
        out = {}
        for arm in ARMS:
            asset = self.base.scene[arm]
            i = asset.find_bodies("gripper")[0][0]
            pose = _arr(asset.data.body_link_pose_w)[0, i]
            out[arm] = pose[:3] - self.origin + quat_to_matrix(pose[3:7])[:, 2] * TCP_OFFSET
        return out

    def physics_step(self) -> None:
        """One physics step: read the last step's blade contacts, apply the cut rule."""
        from pxr import UsdPhysics

        s = self.spec
        self.t += self.dt
        fm = _arr(self.contact.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        tr = _arr(self.bodies.get_transforms())
        blade = tr[s.n_seg + 2]
        bR = quat_to_matrix(blade[3:7])
        edge = np.array([edge_force(FORCE_SIGN * fm[k], bR, s.edge_normal) for k in range(s.n_seg)])
        if s.table_z is not None and blade_low_z(blade[:3] - self.origin, bR, s.blade_size) <= s.table_z + 0.001:
            if not self.cutter.on_table:
                print(f"[cordkit] blade touched the table at t={self.t:.2f}s", flush=True)
            self.cutter.on_table = True
        self.step_peak = max(self.step_peak, float(edge.max(initial=0.0)))
        self.step_neg = max(self.step_neg, float((-edge).max(initial=0.0)))
        self.step_touch = max(self.step_touch, float(np.linalg.norm(fm[s.n_seg:], axis=1).max(initial=0.0)))
        touch = {f"{arm}/{name}": float(np.linalg.norm(fm[s.n_seg + i])) for i, (arm, name, _) in enumerate(self.links)}
        for k in self.cutter.step(edge, touch):
            n = np.asarray(s.edge_normal, float)
            edge_p = blade[:3] + bR @ n * float(np.abs(n) @ np.asarray(s.blade_size)) / 2
            seg = tr[k]
            j = nearer_joint(k, seg[:3], quat_to_matrix(seg[3:7]), edge_p, bR @ np.asarray(s.blade_axis), s.seg_len)
            if self.cutter.cut[j]:
                j = k + 1 if j == k else k
            if self.cutter.cut[j]:
                continue
            point = seg[:3] + quat_to_matrix(seg[3:7])[:, 2] * s.seg_len / 2 * (1 if j == k else -1) - self.origin
            tcp = self.tcps()
            knife = tr[s.n_seg + 1][:3] - self.origin
            holder = min(ARMS, key=lambda a: np.linalg.norm(tcp[a] - knife))
            other = next(a for a in ARMS if a != holder)
            UsdPhysics.Joint(self.joints[j]).GetJointEnabledAttr().Set(False)
            self.cutter.cut[j] = True
            held = float(np.linalg.norm(tr[: s.n_seg + 1, :3] - self.origin - tcp[other], axis=1).min())
            self.cutter.cuts.append(Cut(j, point, float(np.linalg.norm(tcp[other] - point)), self.t, held))
            print(f"[cordkit] cut joint {j} at {np.round(point, 3)} t={self.t:.2f}s force {edge[k]:.1f} N, "
                  f"{np.linalg.norm(tcp[other] - point) * 100:.1f} cm from the {other} gripper, which is {held * 100:.1f} cm "
                  f"from the cord or tag", flush=True)

    def reset(self) -> None:
        """Put every body back where it started, at rest, and re-tie the cord."""
        import torch
        from pxr import UsdPhysics

        for prim in self.joints:
            UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(True)
        idx = np.arange(len(self.names), dtype=np.int32)
        cur = self.bodies.get_transforms()
        if hasattr(cur, "detach"):
            dev = cur.device
            self.bodies.set_transforms(torch.tensor(self.start, dtype=torch.float32, device=dev),
                                       torch.tensor(idx, device=dev))
            self.bodies.set_velocities(torch.zeros((len(idx), 6), dtype=torch.float32, device=dev),
                                       torch.tensor(idx, device=dev))
        else:
            import warp as wp

            dev = getattr(cur, "device", None)
            self.bodies.set_transforms(wp.array(self.start.astype(np.float32), device=dev),
                                       wp.array(idx, dtype=wp.int32, device=dev))
            self.bodies.set_velocities(wp.zeros((len(idx), 6), dtype=wp.float32, device=dev),
                                       wp.array(idx, dtype=wp.int32, device=dev))
        self.cutter.reset()
        self.t = 0.0

    def state(self) -> CordState:
        s = self.spec
        tr = _arr(self.bodies.get_transforms())
        vel = _arr(self.bodies.get_velocities())
        p = tr[:, :3] - self.origin
        tag = p[s.n_seg]
        R = quat_to_matrix(tr[s.n_seg + 1][3:7])
        if self.debug:
            print(f"[cordkit] t={self.t:5.2f} tag {np.round(tag, 3)} knife {np.round(p[s.n_seg + 1], 3)} "
                  f"edge peak {self.step_peak:5.2f} N (opposite {self.step_neg:5.2f}, robot {self.step_touch:5.2f}, "
                  f"max {self.cutter.peak:5.2f}) tcpL {np.round(self.tcps()['left_robot'], 3)} "
                  f"tcpR {np.round(self.tcps()['right_robot'], 3)} blade {np.round(p[s.n_seg + 2], 3)} "
                  f"span {np.linalg.norm(p[s.n_seg] - np.asarray(s.pos)):.4f} cuts "
                  f"{[c.joint for c in self.cutter.cuts]} touched {self.cutter.touched}", flush=True)
        self.step_peak = self.step_neg = self.step_touch = 0.0
        lo, hi = p.min(0), p.max(0)
        return CordState(pos=tag, quat_xyzw=tr[s.n_seg][3:7], lin_vel=vel[s.n_seg, :3], ang_vel=vel[s.n_seg, 3:],
                         low=lo, high=hi, seg_pos=p[:s.n_seg], tag_pos=tag, knife_pos=p[s.n_seg + 1], knife_R=R,
                         knife_vel=vel[s.n_seg + 1], blade_pos=p[s.n_seg + 2], tcp=self.tcps(),
                         cuts=tuple(self.cutter.cuts), touched=self.cutter.touched, blade_on_table=self.cutter.on_table,
                         peak_edge_n=self.cutter.peak)


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        try:
            _RT[name] = _Runtime(base, _SPECS[name])
        except Exception:
            import traceback

            traceback.print_exc()
            raise
    return _RT[name]


# -- hooks ------------------------------------------------------------------------------------------------------

_RESET: list = []


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_cordkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, CordRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._cordkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_cordkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._cordkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_cordkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, *a, **k):
        if self._env is not None:
            for name in _SPECS:
                _runtime(self._base, name).reset()
        obs = reset(self, *a, **k)
        for name in _SPECS:
            _runtime(self._base, name).cutter.reset()  # the settle steps are not part of the trial
            _runtime(self._base, name).t = 0.0
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._ensure_sim, cls.reset, cls._cordkit = _ensure_sim, _reset, True


def _patch_demo(demo) -> None:
    if getattr(demo, "_cordkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._cordkit = _grip_rotation, True


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
