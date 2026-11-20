"""poolkit: the near corner of a 7 ft bar pool table, two regulation 57 mm balls and a 48 in shorty cue for a yam_arena
task file, and a 240 Hz shot monitor.

yam_arena task files only take single rigid primitives with one grippy, non-bouncy material. This module adds:

- :class:`Table`: one fixed (kinematic) body with many box colliders: the bed (slate and cloth) with six pocket holes,
  rubber cushions (bouncy) and wooden rails, each with its own material.
- :class:`Ball`: a 57 mm, 170 g solid ball with the ball's restitution, cloth friction and a rolling-resistance damping.
- :class:`Cue`: one free body, a tapered cue as three colliders (leather tip, shaft, butt).

and hooks yam_arena (without modifying it):

- ``yam_arena.scene.task_object`` builds these with their own USD spawner.
- Before every PHYSICS step (240 Hz, a wrapper on ``InteractiveScene.write_data_to_sim``) a :class:`ShotMonitor`
  reads both balls' poses and their contact forces with the cue, with each robot and with each other (PhysX contact
  reports). A cue strike lasts a few milliseconds, so it is counted at this rate, not per control step.
- ``yam_arena.scene.object_states`` reports a :class:`BallState` for each ball: its pose plus the monitor's record.
- ``yam_arena.scripted.YamScriptedPolicy``: if the task module defines ``demo_controller``, ``yam_scripted`` runs it
  closed loop on each observation and the latest object states, instead of a waypoint plan.
- Each embodiment reset clears the monitor.

The top half (parameters, table geometry, :class:`ShotMonitor`, :func:`shot_score`) is plain NumPy and is what the
tests exercise. Everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState, Sphere

# -- parts (pure Python) -------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class BallParams:
    radius: float = 0.028575  # regulation 2 1/4 in (57.15 mm)
    mass: float = 0.170  # 6 oz (Aramith regulation weight)
    restitution: float = 0.93  # ball on ball (phenolic); ball on cushion uses the cushion's value (min combine)
    friction: float = 0.2  # ball on cloth, sliding (multiply combine: x the cloth's 1.0); ball on ball 0.04
    # Rolling resistance of the cloth as body damping (1/s): a ball rolled at 0.5 m/s stops after about 1 m (v / 0.45).
    linear_damping: float = 0.45
    angular_damping: float = 0.45


@dataclass(frozen=True)
class CueParams:
    """Action ACTO48, a one-piece 48 in shorty cue sold for tight bar spaces: 18 oz, 12 mm tip."""

    length: float = 1.219  # 48 in
    tip_r: float = 0.006  # 12 mm leather tip, domed
    shaft_r: float = 0.0068  # 13.6 mm shaft (the taper is modelled as two steps)
    shaft_len: float = 0.60
    butt_r: float = 0.0145  # 29 mm butt
    mass: float = 0.510  # 18 oz
    tip_friction: float = 3.0  # chalked leather: x ball 0.2 = 0.6
    tip_restitution: float = 0.75
    wood_friction: float = 1.0  # the butt's wrap, where the hand grips
    shaft_friction: float = 0.35  # smooth lacquered shaft (min combine): it slides through a bridge
    grip_patch: float = 0.010  # m: torsional patch radius under a gripper pad (a 20 mm wide pad contact)

    @property
    def com_from_tip(self) -> float:
        """Balance point (m from the tip) of a shaft and butt of uniform density."""
        vs = math.pi * self.shaft_r**2 * self.shaft_len
        lb = self.length - self.shaft_len
        vb = math.pi * self.butt_r**2 * lb
        return (vs * self.shaft_len / 2 + vb * (self.shaft_len + lb / 2)) / (vs + vb)


@dataclass(frozen=True)
class TableGeom:
    """The near-left corner region of a 7 ft bar table (39 x 78 in playing field), world frame (m). The long side runs
    along y, from the target corner at y = ``y_left`` toward -y; the field's far side and right end lie beyond the
    workspace and the cameras. The bed (slate and cloth) stands ``felt_h`` above the station table top."""

    top: float  # the station table top it stands on
    x0: float = 0.200  # playing field near edge (cushion nose)
    depth: float = 0.991  # playing field, x (39 in)
    length: float = 1.981  # playing field, y (78 in)
    y_left: float = 0.550  # the field's left edge (cushion nose)
    felt_h: float = 0.070  # cloth above the station table: the pocket holes are this deep
    cushion_w: float = 0.050  # cushion nose to rail (2 in)
    cushion_h: float = 0.036  # nose height above the cloth (1 7/16 in, 63.5 % of a 2 1/4 in ball)
    rail_w: float = 0.100  # wooden rail top (4 in)
    rail_h: float = 0.044  # rail top above the cloth
    corner: float = 0.085  # corner pocket hole: this square in each field corner (a 4 3/4 in mouth across the jaws)
    side_w: float = 0.130  # side pocket hole: this wide (y, 5 1/8 in mouth) ...
    side_d: float = 0.040  # ... and this deep into the field (x)

    @property
    def x1(self) -> float:
        return self.x0 + self.depth

    @property
    def y1(self) -> float:
        return self.y_left

    @property
    def y0(self) -> float:
        return self.y_left - self.length

    @property
    def ym(self) -> float:
        return self.y_left - self.length / 2

    @property
    def felt(self) -> float:
        return self.top + self.felt_h

    def pockets(self) -> dict[str, tuple[float, float, float, float]]:
        """Each pocket's hole in the slate as (x0, x1, y0, y1)."""
        c, sw, sd, ym = self.corner, self.side_w, self.side_d, self.ym
        x0, x1, y0, y1 = self.x0, self.x1, self.y0, self.y1
        return {
            "near_left": (x0, x0 + c, y1 - c, y1), "far_left": (x1 - c, x1, y1 - c, y1),
            "near_right": (x0, x0 + c, y0, y0 + c), "far_right": (x1 - c, x1, y0, y0 + c),
            "near_side": (x0, x0 + sd, ym - sw / 2, ym + sw / 2), "far_side": (x1 - sd, x1, ym - sw / 2, ym + sw / 2),
        }

    def pocket_mouth(self, name: str) -> np.ndarray:
        """The field corner (or rail midpoint) a pocket sits on: the point the 10 cm partial credit is measured from."""
        xs = {"near": self.x0, "far": self.x1}[name.split("_")[0]]
        ys = {"left": self.y1, "right": self.y0, "side": self.ym}[name.split("_")[1]]
        return np.array([xs, ys])

    def boxes(self) -> list[tuple[str, tuple, tuple]]:
        """(material, centre, size) of every collider, world frame."""
        out = []
        f, top = self.felt, self.top
        c, sw, sd, ym = self.corner, self.side_w, self.side_d, self.ym
        x0, x1, y0, y1 = self.x0, self.x1, self.y0, self.y1
        cw, rw = self.cushion_w, self.rail_w

        def box(mat, xa, xb, ya, yb, za, zb):
            out.append((mat, ((xa + xb) / 2, (ya + yb) / 2, (za + zb) / 2), (xb - xa, yb - ya, zb - za)))

        ox0, ox1, oy0, oy1 = x0 - cw - rw, x1 + cw + rw, y0 - cw - rw, y1 + cw + rw
        zs = top + 0.002  # the slate's underside; pocket floors are the station table
        # slate, leaving the six holes
        box("felt", x0 + c, x1 - c, y0, y1, zs, f)
        for xa, xb, xs in ((x0, x0 + c, x0), (x1 - c, x1, x1)):
            box("felt", xa, xb, y0 + c, ym - sw / 2, zs, f)
            box("felt", xa, xb, ym + sw / 2, y1 - c, zs, f)
            inner = (x0 + sd, x0 + c) if xs == x0 else (x1 - c, x1 - sd)
            box("felt", inner[0], inner[1], ym - sw / 2, ym + sw / 2, zs, f)
        # cushions (gaps at the pockets) and rails
        ch, rh = f + self.cushion_h, f + self.rail_h
        for ya, yb in ((y0 + c, ym - sw / 2), (ym + sw / 2, y1 - c)):
            box("cushion", x0 - cw, x0, ya, yb, zs, ch)
            box("cushion", x1, x1 + cw, ya, yb, zs, ch)
        box("cushion", x0 + c, x1 - c, y0 - cw, y0, zs, ch)
        box("cushion", x0 + c, x1 - c, y1, y1 + cw, zs, ch)
        box("wood", ox0, x0 - cw, oy0, oy1, zs, rh)  # near rail (the cue rests on it)
        box("wood", x1 + cw, ox1, oy0, oy1, zs, rh)
        box("wood", x0 - cw, x1 + cw, oy0, y0 - cw, zs, rh)
        box("wood", x0 - cw, x1 + cw, y1 + cw, oy1, zs, rh)
        # the pocket's back walls behind the cushion gaps (inside faces of the rails), down to the pocket floor
        for xa, xb in ((x0 - cw, x0), (x1, x1 + cw)):
            for ya, yb in ((y0 - cw, y0 + c), (y1 - c, y1 + cw)):
                box("wood", xa, xb, ya, yb, zs, rh)
            box("wood", xa, xb, ym - sw / 2, ym + sw / 2, zs, rh)
        for ya, yb in ((y0 - cw, y0), (y1, y1 + cw)):
            for xa, xb in ((x0, x0 + c), (x1 - c, x1)):
                box("wood", xa, xb, ya, yb, zs, rh)
        return out


# -- the rules (pure Python) ---------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Rules:
    target_pocket: str = "near_left"
    strike_gap_s: float = 0.03  # cue-ball contacts closer than this are one strike
    push_s: float = 0.07  # one cue contact longer than this is a push (17+ frames at 240 Hz): a foul
    near_m: float = 0.10  # object ball at rest within this of the target pocket's mouth: partial credit
    touch_n: float = 1e-3  # N: any contact force above this counts as a touch


class ShotMonitor:
    """The shot's record from contacts and poses, read every physics step.

    Latched over the whole trial: ``strikes`` (separate cue contacts on the cue ball), ``push`` (one cue contact
    lasting over ``push_s``), ``foul`` (a robot touched either ball, or the cue touched the object ball), ``hit``
    (the cue ball touched the object ball), and each ball's pocket (``potted``) or ``off_table``.
    """

    def __init__(self, rules: Rules, table: TableGeom, radius: float, dt: float = 1 / 240):
        self.r, self.t, self.radius, self.dt = rules, table, radius, dt
        self.holes = table.pockets()
        self.reset()

    def reset(self) -> None:
        self.strikes = 0
        self.contact_s = 0.0  # length of the current cue-ball contact
        self.gap_s = 1e9  # time since the last cue-ball contact
        self.push = False
        self.foul = False
        self.foul_reason = ""
        self.hit = False
        self.potted: dict[str, str] = {}
        self.off_table: set[str] = set()
        self.peak_cb_speed = 0.0

    def _foul(self, why: str) -> None:
        if not self.foul:
            self.foul, self.foul_reason = True, why

    def where(self, pos) -> str:
        """'' on the table, a pocket name, or 'off' (fell off the table)."""
        x, y, z = pos
        if z > self.t.felt + 0.75 * self.radius:
            return ""
        for name, (xa, xb, ya, yb) in self.holes.items():
            # sunk more than a quarter of its radius below the cloth level inside a hole (it may rest on another ball)
            if xa - 0.01 <= x <= xb + 0.01 and ya - 0.01 <= y <= yb + 0.01 and z < self.t.felt + 0.75 * self.radius:
                return name
        return "off" if z < self.t.felt - 0.03 else ""

    def step(self, balls: dict, cue_cb: float, cue_ob: float, robot_cb: float, robot_ob: float, cb_ob: float,
             cb_speed: float = 0.0) -> None:
        """One physics step. ``balls``: {"cue_ball": pos, "object_ball": pos}; the rest are contact force norms (N)."""
        r = self.r
        if robot_cb > r.touch_n:
            self._foul("a robot touched the cue ball")
        if robot_ob > r.touch_n:
            self._foul("a robot touched the object ball")
        if cue_ob > r.touch_n:
            self._foul("the cue touched the object ball")
        if cb_ob > r.touch_n:
            self.hit = True
        if cue_cb > r.touch_n:
            if self.contact_s == 0.0 and self.gap_s >= r.strike_gap_s:
                self.strikes += 1
            self.contact_s += self.dt
            self.gap_s = 0.0
            if self.contact_s > r.push_s:
                self.push = True
        else:
            self.contact_s = 0.0
            self.gap_s += self.dt
        self.peak_cb_speed = max(self.peak_cb_speed, cb_speed)
        for name, pos in balls.items():
            if name in self.potted or name in self.off_table:
                continue
            w = self.where(pos)
            if w == "off":
                self.off_table.add(name)
            elif w:
                self.potted[name] = w


def shot_score(m: ShotMonitor, ob_pos, ob_resting: bool) -> float:
    """1.0: the object ball in the target pocket, the cue ball still on the table, one clean strike.
    0.5: the object ball in another pocket, or at rest within ``near_m`` of the target pocket's mouth.
    0.25: the cue ball hit the object ball. A scratch (cue ball potted or off the table) caps the score at 0.25.
    0: no strike, more than one strike, a push, or a foul (a robot touched a ball, the cue touched the object ball)."""
    r = m.r
    if m.strikes != 1 or m.push or m.foul or not m.hit:
        return 0.0
    scratch = "cue_ball" in m.potted or "cue_ball" in m.off_table
    pocket = m.potted.get("object_ball")
    if pocket == r.target_pocket:
        s = 1.0
    elif pocket is not None:
        s = 0.5
    elif ("object_ball" not in m.off_table and ob_resting
          and float(np.linalg.norm(np.asarray(ob_pos)[:2] - m.t.pocket_mouth(r.target_pocket))) <= r.near_m):
        s = 0.5
    else:
        s = 0.25
    return min(s, 0.25) if scratch else s


# -- task-file object specs ----------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Table(Box):
    geom: TableGeom = None  # type: ignore[assignment]
    felt_color: tuple = (0.10, 0.40, 0.20)
    cushion_color: tuple = (0.08, 0.33, 0.16)
    wood_color: tuple = (0.30, 0.17, 0.08)


@dataclass(frozen=True)
class Ball(Sphere):
    params: BallParams = field(default_factory=BallParams)


@dataclass(frozen=True)
class Cue(Box):
    """The cue, its balance point at ``pos``, the tip toward +x before ``yaw``."""

    params: CueParams = field(default_factory=CueParams)
    tip_color: tuple = (0.15, 0.35, 0.75)
    shaft_color: tuple = (0.90, 0.78, 0.55)
    butt_color: tuple = (0.10, 0.06, 0.04)


@dataclass(frozen=True)
class BallState(ObjectState):
    strikes: int = 0
    push: bool = False
    foul: bool = False
    foul_reason: str = ""
    hit: bool = False
    pocket: str = ""
    off_table: bool = False
    cue_pos: np.ndarray | None = None
    cue_quat_xyzw: np.ndarray | None = None


_SPECS: dict = {}
_CFG: dict = {}
_MON: list[ShotMonitor] = []
_RUN: dict = {}
LATEST: dict = {}  # the last object states, for the scripted demo


def configure(table: Table, cue_ball: Ball, object_ball: Ball, cue: Cue, rules: Rules) -> None:
    _SPECS.clear()
    for s in (table, cue_ball, object_ball, cue):
        _SPECS[s.name] = s
    _CFG.update(table=table.name, cb=cue_ball.name, ob=object_ball.name, cue=cue.name, rules=rules)
    _MON[:] = [ShotMonitor(rules, table.geom, cue_ball.params.radius)]


def monitor() -> ShotMonitor:
    return _MON[0]


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def cue_tip(cue_pos, cue_quat, p: CueParams) -> np.ndarray:
    """World position of the tip's end."""
    return np.asarray(cue_pos) + quat_matrix(cue_quat)[:, 0] * p.com_from_tip


# -- Isaac Sim side ------------------------------------------------------------------------------------------------


def _material(path: str, friction: float, restitution: float = 0.0, fcombine="average", rcombine="average") -> str:
    import isaaclab.sim as sim_utils

    cfg = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction, restitution=restitution,
                                         friction_combine_mode=fcombine, restitution_combine_mode=rcombine)
    cfg.func(path, cfg)
    return path


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.object_name]
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    looks = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def collider(geom, color, mat, offset=0.002, patch=0.0) -> None:
        g = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(g)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(g)
        cp.CreateContactOffsetAttr(offset)
        cp.CreateRestOffsetAttr(0.0)
        if patch:  # torsional friction: a rubber pad on a round cue grips over a patch this wide, not at a point
            cp.CreateTorsionalPatchRadiusAttr(patch)
            cp.CreateMinTorsionalPatchRadiusAttr(patch)
        bind_physics_material(str(g.GetPath()), mat, stage=stage)
        bind_visual_material(str(g.GetPath()), look(tuple(color)), stage=stage)

    if isinstance(spec, Table):
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)

        mats = {
            "felt": _material(f"{prim_path}/Looks/felt", 1.0, 0.0),
            "cushion": _material(f"{prim_path}/Looks/cushion", 1.0, 0.75),
            "wood": _material(f"{prim_path}/Looks/wood", 0.5, 0.5),
        }
        colors = {"felt": spec.felt_color, "cushion": spec.cushion_color, "wood": spec.wood_color}
        origin = np.asarray(spec.pos, float)
        for i, (mat, centre, size) in enumerate(spec.geom.boxes()):
            cube = UsdGeom.Cube.Define(stage, f"{prim_path}/{mat}_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.asarray(centre) - origin)))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collider(cube, colors[mat], mats[mat])
        return prim

    if isinstance(spec, Ball):
        p = spec.params
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(p.mass)
        ixx = 0.4 * p.mass * p.radius**2  # solid ball
        UsdPhysics.MassAPI(prim).CreateDiagonalInertiaAttr(Gf.Vec3f(ixx, ixx, ixx))
        rb.CreateLinearDampingAttr(p.linear_damping)
        rb.CreateAngularDampingAttr(p.angular_damping)
        rb.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        mat = _material(f"{prim_path}/Looks/ball_mat", p.friction, p.restitution, "multiply", "min")
        sph = UsdGeom.Sphere.Define(stage, f"{prim_path}/shell")
        sph.CreateRadiusAttr(p.radius)
        collider(sph, spec.color, mat)
        return prim

    p = spec.params  # the cue
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(p.mass)
    UsdPhysics.MassAPI(prim).CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))  # the body origin is the balance point
    rb.CreateEnableCCDAttr(True)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    tipm = _material(f"{prim_path}/Looks/tip", p.tip_friction, p.tip_restitution, "multiply", "min")
    wood = _material(f"{prim_path}/Looks/wood", p.wood_friction, 0.6)
    shaft = _material(f"{prim_path}/Looks/shaft", p.shaft_friction, 0.6, "min")
    a = p.com_from_tip  # tip end at local x = +a
    tip = UsdGeom.Sphere.Define(stage, f"{prim_path}/tip")
    tip.CreateRadiusAttr(p.tip_r)
    tip.AddTranslateOp().Set(Gf.Vec3d(a - p.tip_r, 0.0, 0.0))
    collider(tip, spec.tip_color, tipm)
    for name, x_a, x_b, r, color, mat in (("shaft", a - p.shaft_len, a - p.tip_r, p.shaft_r, spec.shaft_color, shaft),
                                          ("butt", a - p.length, a - p.shaft_len, p.butt_r, spec.butt_color, wood)):
        cyl = UsdGeom.Cylinder.Define(stage, f"{prim_path}/{name}")
        cyl.CreateAxisAttr("X")
        cyl.CreateRadiusAttr(r)
        cyl.CreateHeightAttr(x_b - x_a)
        cyl.AddTranslateOp().Set(Gf.Vec3d((x_a + x_b) / 2, 0.0, 0.0))
        collider(cyl, color, mat, patch=p.grip_patch)
    return prim


def _arena_object(spec):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(_spawn)
        object_name: str = ""

    return Object(
        name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(object_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _body_path(asset) -> str:
    return list(asset.root_physx_view.prim_paths)[0]


def _link_paths(robot) -> list[str]:
    view = getattr(robot, "root_physx_view", None) or robot.root_view
    return list(view.link_paths[0])


class _Runtime:
    def __init__(self, base):
        from isaaclab_physx.physics import PhysxManager

        self.base = base
        sc = base.scene
        self.cb, self.ob, self.cue = sc[_CFG["cb"]], sc[_CFG["ob"]], sc[_CFG["cue"]]
        view = PhysxManager.get_physics_sim_view()
        self.left = _link_paths(sc["left_robot"])
        self.right = _link_paths(sc["right_robot"])
        cue_p, cb_p, ob_p = _body_path(self.cue), _body_path(self.cb), _body_path(self.ob)
        # filters per ball: [cue, other ball, left robot links..., right robot links...]
        self.cb_view = view.create_rigid_contact_view([cb_p], filter_patterns=[[cue_p, ob_p, *self.left, *self.right]],
                                                      max_contact_data_count=256)
        self.ob_view = view.create_rigid_contact_view([ob_p], filter_patterns=[[cue_p, cb_p, *self.left, *self.right]],
                                                      max_contact_data_count=256)
        self.origin = _np(base.scene.env_origins)[0]
        self.dt = float(base.physics_dt)
        self.steps = 0
        self.log = []

    def pos(self, asset):
        return _np(asset.data.root_link_pose_w)[0][:3] - self.origin

    def forces(self, view) -> np.ndarray:
        f = _np(view.get_contact_force_matrix(dt=self.dt)).reshape(-1, 3)
        return np.linalg.norm(f, axis=1)


def _runtime(base) -> _Runtime:
    rt = _RUN.get("rt")
    if rt is None or rt.base is not base:
        rt = _RUN["rt"] = _Runtime(base)
    return rt


def _physics(base) -> None:
    rt = _runtime(base)
    rt.steps += 1
    if _RUN.get("settling"):
        return
    fc, fo = rt.forces(rt.cb_view), rt.forces(rt.ob_view)
    cbp, obp = rt.pos(rt.cb), rt.pos(rt.ob)
    v = _np(rt.cb.data.root_vel_w)[0][:3]
    monitor().step({"cue_ball": cbp, "object_ball": obp}, cue_cb=fc[0], cue_ob=fo[0], robot_cb=float(fc[2:].max()),
                   robot_ob=float(fo[2:].max()), cb_ob=max(fc[1], fo[1]), cb_speed=float(np.linalg.norm(v)))
    if _LOG_PATH:
        cue = _np(rt.cue.data.root_link_pose_w)[0]
        cv = _np(rt.cue.data.root_vel_w)[0][:3]
        m = monitor()
        g = []
        for side in ("left_robot", "right_robot"):
            arm = base.scene[side]
            gid = arm.find_bodies("gripper")[0][0]
            gp = _np(arm.data.body_link_pose_w)[0, gid]
            g.append(gp[:3] - rt.origin + quat_matrix(gp[3:7]) @ np.array([0.0, 0.0, 0.134]))
        rt.log.append(np.r_[rt.steps * rt.dt, cbp, v, obp, cue[:3] - rt.origin, cue[3:7], cv, fc[0], fc[1],
                            fc[2:].max(), fo[0], fo[2:].max(), m.strikes, m.hit, m.foul, g[0], g[1]])
        _RUN["log"] = rt.log
        if rt.steps % 240 == 0:
            np.save(_LOG_PATH, np.array(rt.log))


_LOG_PATH = os.environ.get("POOL_LOG", "")


def _ball_state(environment, name, plain) -> BallState:
    base = environment.unwrapped
    rt = _runtime(base)
    s = plain[name]
    m = monitor()
    cue = _np(rt.cue.data.root_link_pose_w)[0]
    return BallState(pos=s.pos, quat_xyzw=s.quat_xyzw, lin_vel=s.lin_vel, ang_vel=s.ang_vel, low=s.low, high=s.high,
                     strikes=m.strikes, push=m.push, foul=m.foul, foul_reason=m.foul_reason, hit=m.hit,
                     pocket=m.potted.get(name, ""), off_table=name in m.off_table,
                     cue_pos=cue[:3] - rt.origin, cue_quat_xyzw=cue[3:7])


# -- hooking yam_arena ---------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_poolkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def kit_task_object(spec):
        if isinstance(spec, Table | Ball | Cue):
            return _arena_object(spec)
        return task_object(spec)

    def kit_object_states(environment, names):
        out = object_states(environment, names)
        for key in ("cb", "ob"):
            if _CFG.get(key) in out:
                out[_CFG[key]] = _ball_state(environment, _CFG[key], out)
        LATEST.clear()
        LATEST.update(out)
        return out

    scene.task_object, scene.object_states, scene._poolkit = kit_task_object, kit_object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_poolkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        _physics(base)
        return write(*a, **k)

    isc.write_data_to_sim, isc._poolkit = write_data_to_sim, True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_poolkit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def kit_ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def kit_reset(self, *a, **k):
        _RUN["settling"] = True
        try:
            out = reset(self, *a, **k)
        finally:
            _RUN["settling"] = False
        monitor().reset()
        LATEST.clear()
        return out

    cls._ensure_sim, cls.reset, cls._poolkit = kit_ensure_sim, kit_reset, True


def _patch_scripted(scripted) -> None:
    """``yam_scripted`` runs the task module's ``demo_controller`` (if any) closed loop on each observation."""
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_poolkit", False):
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

        data, done = ctl.act(observation, LATEST)
        meta = {"request_stop": True} if done else {}
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1.0 / self._dt)

    cls.reset, cls.act, cls._poolkit = kit_reset, kit_act, True


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
