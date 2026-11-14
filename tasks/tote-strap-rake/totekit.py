"""totekit: a shelf, a storage tote with a hanging strap, and a hand rake whose tine the strap's ring is caught on.

yam_arena task files only take single primitives. This module adds one object, :class:`ToteRig`, spawned as a fixed
(``BASE``) Arena asset holding:

- ``shelf``: static colliders: a steel deck on four corner posts, standing on the table.
- ``tote``: one rigid body (bottom, four walls and a hand-hold ledge along the top of each end wall).
- ``strap0`` .. ``strap3``: four rigid webbing links joined by spherical joints, the first one to the tote's back wall,
  the last one to ``ring``, a steel ring (one rigid body of four bars).
- ``rake``: one rigid body: handle, neck, head and three upright tines.

Before every physics step (240 Hz, via a wrapper on ``InteractiveScene.write_data_to_sim``) the kit reads the bodies'
poses and latches: ``rake_off`` (a point of the rake went more than ``EDGE_M`` past an edge of the deck, or below the
deck top), ``dragged`` (the rake moved more than ``DRAG_M`` from where it started while the ring was round a tine),
``tote_dropped`` (the tote fell faster than ``DROP_V``), ``tote_floor`` (it went below the table top).

``object_states`` reports a :class:`ToteState`. Importing the module also hooks ``yam_arena.scripted`` so that
``yam_scripted`` plays the task file's closed-loop ``controller`` (see ``totedemo.py``). yam_arena itself is not
modified. Pure Python at import; Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, ObjectState

ARMS = ("left_robot", "right_robot")
FINGER_OPEN = -0.04695  # yam_arena.constants.FINGER_OPEN
TCP_OFFSET = 0.134  # yam_arena.constants.TCP_OFFSET, on the gripper link's z axis
TOP = TABLE_TOP["station"]

# -- layout (env frame: x forward, y left, z up; m) ------------------------------------------------------------------
# Shelf: a steel deck 15 cm above the table on four 25 mm posts.
DECK_TOP = TOP + 0.15
DECK_T = 0.015
DECK_X = (0.36, 0.66)  # front edge, back edge
DECK_Y = (-0.32, 0.44)  # right edge, left edge
POST = 0.025
# Tote: polypropylene, 26 cm (y) x 18 cm (x) x 10 cm, 5 mm walls; a 3.5 cm hand-hold ledge, 10 cm long, along the top
# of each end wall.
TOTE_L, TOTE_W, TOTE_H, WALL = 0.26, 0.18, 0.10, 0.005
LEDGE_OUT, LEDGE_LEN, LEDGE_T = 0.035, 0.10, 0.006
TOTE_MASS = 0.40
TOTE_XY = (0.47, -0.06)  # centre of the tote's footprint at the start (seed shift 0)
# Strap: 25 mm polypropylene webbing, 5 links of 3.95 cm (19.75 cm, 1 mm more than the gap: taut), sewn to the back wall 1 cm under the rim, 1 cm from the
# tote's left end; it ends in a 45 mm steel ring (6 mm bar, 31 mm opening).
LINK_L, LINK_W, LINK_T, LINK_MASS = 0.0395, 0.025, 0.004, 0.005
N_LINKS = 5
ANCHOR_LOCAL = (TOTE_W / 2, TOTE_L / 2 - 0.01, TOTE_H / 2 - 0.01)  # in the tote's body frame (its box centre)
RING_OUT, RING_BAR, RING_T, RING_MASS = 0.045, 0.007, 0.006, 0.030
RING_HALF_OPEN = RING_OUT / 2 - RING_BAR  # 15.5 mm
# Rake: a steel three-tine hand rake lying on its back, tines up, along y (handle to the left), its tines in a row
# along x, 3 cm apart; the ring is round the middle tine.
RAKE_TINE_Y = 0.23
RAKE_TINE_X = (0.42, 0.45, 0.48)
HEAD = (0.08, 0.02, 0.025)  # x, y, z size of the head (the tines' roots; the ring rests on it)
NECK = (0.015, 0.03, 0.010)
HANDLE = (0.024, 0.14, 0.024)  # wooden handle
TINE_R, TINE_H = 0.004, 0.045  # tines rise 4.5 cm above the head
RAKE_MASS = 0.22
CAUGHT_TINE = 1  # the middle tine

# -- scoring thresholds that the kit latches ---------------------------------------------------------------------------
EDGE_M = 0.01  # a rake point more than 1 cm past a deck edge: off the shelf
BELOW_M = 0.01  # a rake point more than 1 cm below the deck top: off the shelf
DRAG_M = 0.03  # the rake moved 3 cm while the ring was round a tine: dragged by the strap
DROP_V = 0.6  # m/s downward: the tote fell (a 2 cm free fall)
SHIFT_X, SHIFT_Y = 0.02, 0.03  # seeded shift of tote, strap, ring and rake together, uniform within +-


@dataclass(frozen=True)
class ToteRig(Box):
    """The whole rig. ``pos``/``size`` only describe a bounding box for yam_arena's checks."""

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a ToteRig is a fixed asset (mass None); its bodies carry their own")


def shift_for_seed(seed: int | None) -> np.ndarray:
    """(dx, dy) by which the trial's seed moves the tote, strap, ring and rake together (the shelf stays)."""
    if seed is None:
        return np.zeros(2)
    rng = np.random.default_rng(int(seed))
    return np.array([rng.uniform(-SHIFT_X, SHIFT_X), rng.uniform(-SHIFT_Y, SHIFT_Y)])


# -- geometry in body frames -------------------------------------------------------------------------------------------


def tote_boxes():
    """(centre, size) of the tote's parts in its body frame (origin at the centre of its outer box)."""
    w, l, h, t = TOTE_W, TOTE_L, TOTE_H, WALL
    out = [((0, 0, -h / 2 + t / 2), (w, l, t)),  # bottom
           ((w / 2 - t / 2, 0, 0), (t, l, h)), ((-w / 2 + t / 2, 0, 0), (t, l, h)),  # back, front walls
           ((0, l / 2 - t / 2, 0), (w - 2 * t, t, h)), ((0, -l / 2 + t / 2, 0), (w - 2 * t, t, h))]  # end walls
    for s in (1, -1):  # ledges, flush with the rim
        out.append(((0, s * (l / 2 + LEDGE_OUT / 2), h / 2 - LEDGE_T / 2), (LEDGE_LEN, LEDGE_OUT, LEDGE_T)))
    return out


def ring_boxes():
    o, b = RING_OUT, RING_BAR
    c = o / 2 - b / 2
    return [((0, c, 0), (o, b, RING_T)), ((0, -c, 0), (o, b, RING_T)),
            ((c, 0, 0), (b, o - 2 * b, RING_T)), ((-c, 0, 0), (b, o - 2 * b, RING_T))]


RING_ATTACH = (0.0, -(RING_OUT / 2 - RING_BAR / 2), 0.0)  # the strap is sewn round the ring's -y bar

# Rake body frame: origin at the head bar's centre, at the deck top (z 0 = the rake's underside).
_HX = RAKE_TINE_X[1]


def rake_parts():
    """(kind, centre, size) in the rake's body frame: boxes (size x, y, z) and tines (cylinders, radius, height)."""
    hz = HEAD[2]
    neck_y = HEAD[1] / 2 + NECK[1] / 2
    handle_y = HEAD[1] / 2 + NECK[1] + HANDLE[1] / 2
    parts = [("box", (0, 0, hz / 2), HEAD), ("box", (0, neck_y, NECK[2] / 2), NECK),
             ("box", (0, handle_y, HANDLE[2] / 2), HANDLE)]
    for x in RAKE_TINE_X:
        parts.append(("tine", (x - _HX, 0, hz + TINE_H / 2), (TINE_R, TINE_H)))
    return parts


def rake_points() -> np.ndarray:
    """Rake points (body frame) checked against the deck's edges: the bottom corners of head and handle, the tine tips."""
    pts = []
    for kind, c, s in rake_parts():
        if kind == "box":
            for sx in (-1, 1):
                for sy in (-1, 1):
                    pts.append((c[0] + sx * s[0] / 2, c[1] + sy * s[1] / 2, 0.0))
        else:
            pts.append((c[0], c[1], c[2] + s[1] / 2))
    return np.array(pts)


def tines_local() -> np.ndarray:
    """(3, 2, 3): base and tip of each tine in the rake's body frame."""
    hz = HEAD[2]
    return np.array([[(x - _HX, 0, hz), (x - _HX, 0, hz + TINE_H)] for x in RAKE_TINE_X])


def start_poses(shift=(0.0, 0.0)):
    """World positions (env frame) and XYZW rotations of every body at the start, the rig shifted by ``shift``."""
    dx, dy = shift
    tote = np.array([TOTE_XY[0] + dx, TOTE_XY[1] + dy, DECK_TOP + TOTE_H / 2 + 0.0005])
    rake = np.array([_HX + dx, RAKE_TINE_Y + dy, DECK_TOP + 0.0005])
    ring = np.array([RAKE_TINE_X[CAUGHT_TINE] + dx, RAKE_TINE_Y + dy, DECK_TOP + HEAD[2] + RING_T / 2 + 0.001])
    anchor = tote + np.array(ANCHOR_LOCAL)
    end = ring + np.array(RING_ATTACH)
    d = end - anchor
    n = np.linalg.norm(d)
    # The strap is laid in a shallow V when it is longer than the gap (it is: slack of a few mm).
    pts = _chain(anchor, end, N_LINKS, LINK_L)
    links = []
    for i in range(N_LINKS):
        a, b = pts[i], pts[i + 1]
        links.append(((a + b) / 2, _frame_quat(b - a)))
    ident = np.array([0.0, 0.0, 0.0, 1.0])
    return {"tote": (tote, ident), "rake": (rake, ident), "ring": (ring, ident),
            **{f"strap{i}": links[i] for i in range(N_LINKS)}, "_gap": n}


def _chain(a, b, n, l):
    """n+1 points from a to b, n segments of length l, the slack taken up as an upward bow in the middle."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = np.linalg.norm(b - a)
    if n * l <= d:
        return [a + (b - a) * i / n for i in range(n + 1)]
    # one interior lift h at the middle joint: two straight halves, each of length n*l/2
    half = n * l / 2
    h = math.sqrt(max(half**2 - (d / 2) ** 2, 0.0))
    mid = (a + b) / 2 + np.array([0, 0, h])
    k = n // 2
    return [a + (mid - a) * i / k for i in range(k + 1)] + [mid + (b - mid) * i / (n - k) for i in range(1, n - k + 1)]


def _frame_quat(d) -> np.ndarray:
    """XYZW rotation whose x axis points along d and whose y axis is horizontal (the webbing lies flat)."""
    x = np.asarray(d, float) / np.linalg.norm(d)
    y = np.cross([0.0, 0.0, 1.0], x)
    y = y / np.linalg.norm(y) if np.linalg.norm(y) > 1e-9 else np.array([0.0, 1.0, 0.0])
    z = np.cross(x, y)
    return matrix_to_quat(np.column_stack([x, y, z]))


def matrix_to_quat(R) -> np.ndarray:
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- pure checks (used by the latches and by task.py) -------------------------------------------------------------------


def ring_on_tine(ring_pos, ring_R, rake_pos, rake_R) -> bool:
    """True when some tine passes through the ring's opening: the tine's axis crosses the ring's mid-plane inside the
    31 mm opening."""
    for base, tip in tines_local():
        a = ring_R.T @ (rake_pos + rake_R @ base - ring_pos)
        b = ring_R.T @ (rake_pos + rake_R @ tip - ring_pos)
        if (a[2] > 0) == (b[2] > 0) or abs(b[2] - a[2]) < 1e-9:
            continue  # the tine does not reach through the ring's plane
        p = a + (b - a) * (-a[2] / (b[2] - a[2]))
        if abs(p[0]) < RING_HALF_OPEN and abs(p[1]) < RING_HALF_OPEN:
            return True
    return False


def rake_world_points(rake_pos, rake_R) -> np.ndarray:
    return np.asarray(rake_pos) + rake_points() @ np.asarray(rake_R).T


def rake_past_edge(pts) -> bool:
    """Some rake point more than EDGE_M past a deck edge (front, back or either side), or BELOW_M under the deck top."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    return bool((x < DECK_X[0] - EDGE_M).any() or (x > DECK_X[1] + EDGE_M).any() or (y < DECK_Y[0] - EDGE_M).any()
                or (y > DECK_Y[1] + EDGE_M).any() or (z < DECK_TOP - BELOW_M).any())


def tote_corners(pos, R) -> np.ndarray:
    """(8, 3): the corners of the tote's outer box (ledges excluded), bottom four first."""
    c = [(sx * TOTE_W / 2, sy * TOTE_L / 2, sz * TOTE_H / 2) for sz in (-1, 1) for sx in (-1, 1) for sy in (-1, 1)]
    return np.asarray(pos) + np.array(c) @ np.asarray(R).T


def gap_to_tote(p, pos, R) -> float:
    """Distance from a point to the tote's outer box including its ledges (0 inside)."""
    q = np.asarray(R).T @ (np.asarray(p, float) - np.asarray(pos))
    half = np.array([TOTE_W / 2, TOTE_L / 2 + LEDGE_OUT, TOTE_H / 2])
    return float(np.linalg.norm(np.maximum(np.abs(q) - half, 0.0)))


@dataclass(frozen=True)
class ToteState(ObjectState):
    """The rig after a control step (env frame). ``pos``/``quat_xyzw``/velocities are the tote's."""

    tote_R: np.ndarray = None
    corners: np.ndarray = None  # (8, 3), bottom four first
    ring_pos: np.ndarray = None
    ring_caught: bool = True  # the ring is round a tine now
    rake_pos: np.ndarray = None
    rake_pts: np.ndarray = None
    rake_moved: float = 0.0  # horizontal distance of the rake's head from its start
    rake_vel: float = 0.0
    rake_off: bool = False  # latched
    dragged: bool = False  # latched
    tote_dropped: bool = False  # latched
    tote_floor: bool = False  # latched
    pads: dict = field(default_factory=dict)  # arm -> (3, 3) finger pad tips and tool point
    open_frac: dict = field(default_factory=dict)  # arm -> 0 (closed) .. 1 (open)


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, ToteRig] = {}
_PATHS: dict[str, str] = {}
BODIES = ("tote", *[f"strap{i}" for i in range(N_LINKS)], "ring", "rake")
COLORS = {"deck": (0.62, 0.64, 0.66), "post": (0.30, 0.31, 0.33), "tote": (0.12, 0.33, 0.70),
          "strap": (0.96, 0.46, 0.05), "ring": (0.80, 0.81, 0.83), "head": (0.10, 0.38, 0.16),
          "handle": (0.80, 0.70, 0.54)}
FRICTION = {"deck": 0.4, "tote": 0.8, "strap": 0.6, "ring": 0.4, "rake": 0.4}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    _PATHS.setdefault(cfg.rig_name, prim_path)
    stage = get_current_stage()
    dbl = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    looks, mats = {}, {}

    def material(name) -> str:
        if name not in mats:
            mats[name] = f"{prim_path}/Looks/m_{name}"
            mu = FRICTION[name]
            m = sim_utils.RigidBodyMaterialCfg(static_friction=mu, dynamic_friction=mu)
            m.func(mats[name], m)
        return mats[name]

    def look(name) -> str:
        if name not in looks:
            looks[name] = f"{prim_path}/Looks/c_{name}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(COLORS[name]), roughness=0.5)
            vis.func(looks[name], vis)
        return looks[name]

    def collider(prim, mat, col) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        c.CreateContactOffsetAttr(0.002)
        c.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material(mat), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(col), stage=stage)

    def cube(parent, name, centre, size, mat, col):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        collider(c.GetPrim(), mat, col)

    def body(name, pos, quat, mass):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*map(float, pos)))
        x, y, z, w = map(float, quat)
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(w, x, y, z))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(8)
        rb.CreateAngularDampingAttr(0.5)
        rb.CreateLinearDampingAttr(0.05)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        return path

    # shelf: static
    shelf = f"{prim_path}/shelf"
    UsdGeom.Xform.Define(stage, shelf)
    cube(shelf, "deck", ((DECK_X[0] + DECK_X[1]) / 2, (DECK_Y[0] + DECK_Y[1]) / 2, DECK_TOP - DECK_T / 2),
         (DECK_X[1] - DECK_X[0], DECK_Y[1] - DECK_Y[0], DECK_T), "deck", "deck")
    ph = DECK_TOP - DECK_T - TOP
    for i, (px, py) in enumerate([(DECK_X[a] + s * POST / 2, DECK_Y[b] + t * POST / 2)
                                  for a, s in ((0, 1), (1, -1)) for b, t in ((0, 1), (1, -1))]):
        cube(shelf, f"post{i}", (px, py, TOP + ph / 2), (POST, POST, ph), "deck", "post")

    poses = start_poses()
    p, q = poses["tote"]
    tote = body("tote", p, q, TOTE_MASS)
    for i, (c, s) in enumerate(tote_boxes()):
        cube(tote, f"part{i}", c, s, "tote", "tote")
    links = []
    for i in range(N_LINKS):
        p, q = poses[f"strap{i}"]
        links.append(body(f"strap{i}", p, q, LINK_MASS))
        cube(links[-1], "web", (0, 0, 0), (LINK_L, LINK_W, LINK_T), "strap", "strap")
    p, q = poses["ring"]
    ring = body("ring", p, q, RING_MASS)
    for i, (c, s) in enumerate(ring_boxes()):
        cube(ring, f"bar{i}", c, s, "ring", "ring")
    p, q = poses["rake"]
    rake = body("rake", p, q, RAKE_MASS)
    for i, (kind, c, s) in enumerate(rake_parts()):
        if kind == "box":
            cube(rake, f"part{i}", c, s, "rake", "handle" if s == HANDLE else "head")
        else:
            cyl = UsdGeom.Cylinder.Define(stage, f"{rake}/tine{i}")
            cyl.CreateAxisAttr("Z")
            cyl.CreateRadiusAttr(s[0])
            cyl.CreateHeightAttr(s[1])
            cyl.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            collider(cyl.GetPrim(), "rake", "head")

    # joints: tote -> strap0 -> ... -> strap3 -> ring
    def joint(name, b0, b1, p0, q0, p1, q1):
        j = UsdPhysics.SphericalJoint.Define(stage, f"{prim_path}/{name}")
        j.CreateBody0Rel().SetTargets([b0])
        j.CreateBody1Rel().SetTargets([b1])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, p0)))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float, p1)))
        for attr, qq in ((j.CreateLocalRot0Attr(), q0), (j.CreateLocalRot1Attr(), q1)):
            x, y, z, w = map(float, qq)
            attr.Set(Gf.Quatf(w, x, y, z))
        j.CreateAxisAttr("X")
        j.CreateConeAngle0LimitAttr(100.0)
        j.CreateConeAngle1LimitAttr(100.0)
        j.CreateCollisionEnabledAttr(False)

    ident = (0.0, 0.0, 0.0, 1.0)
    half = (LINK_L / 2, 0, 0)
    joint("j_tote", tote, links[0], ANCHOR_LOCAL, poses["strap0"][1], (-LINK_L / 2, 0, 0), ident)
    for i in range(N_LINKS - 1):
        qa, qb = poses[f"strap{i}"][1], poses[f"strap{i + 1}"][1]
        rel = matrix_to_quat(quat_to_matrix(qa).T @ quat_to_matrix(qb))  # strap{i+1}'s frame in strap{i}'s
        joint(f"j_strap{i}", links[i], links[i + 1], half, rel, (-LINK_L / 2, 0, 0), ident)
    joint("j_ring", links[-1], ring, half, ident, RING_ATTACH, poses[f"strap{N_LINKS - 1}"][1])
    return root.GetPrim()


def _arena_object(spec: ToteRig):
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


def _like(ref, data: np.ndarray, dtype="float32"):
    if hasattr(ref, "detach"):
        import torch

        return torch.tensor(data, dtype=getattr(torch, dtype), device=ref.device)
    import warp as wp

    return wp.array(data.astype(dtype), dtype=getattr(wp, dtype), device=getattr(ref, "device", None))


class _Runtime:
    """PhysX view and latches for one rig (env 0)."""

    def __init__(self, base, spec: ToteRig):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.view = view.create_rigid_body_view([f"{root}/{b}" for b in BODIES])
        self.idx = {b: i for i, b in enumerate(BODIES)}
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("TOTE_DEBUG"))
        self.n = 0
        self.shift = np.zeros(2)
        self.pad_ids = {arm: [base.scene[arm].find_bodies(b)[0][0] for b in ("tip_left", "tip_right", "gripper")]
                        for arm in ARMS}
        self.finger_ids = {arm: base.scene[arm].find_joints(["joint7", "joint8"])[0] for arm in ARMS}
        self.clear_latches()

    def clear_latches(self) -> None:
        self.rake_off = self.dragged = self.tote_dropped = self.tote_floor = False
        self.rake_start = None
        self.caught = True
        self.n = 0

    def reset(self, seed) -> None:
        """Put every body back at its start, shifted by the seed, at rest."""
        self.shift = shift_for_seed(seed)
        poses = start_poses(self.shift)
        tr = np.zeros((len(BODIES), 7))
        for b, i in self.idx.items():
            p, q = poses[b]
            tr[i, :3], tr[i, 3:] = np.asarray(p) + self.origin, q
        cur = self.view.get_transforms()
        ids = np.arange(len(BODIES), dtype=np.int32)
        self.view.set_transforms(_like(cur, tr), _like(cur, ids, "int32"))
        self.view.set_velocities(_like(cur, np.zeros((len(BODIES), 6))), _like(cur, ids, "int32"))
        self.clear_latches()
        print(f"[totekit] seed {seed}: rig shifted {self.shift[0] * 100:+.1f} cm in x, {self.shift[1] * 100:+.1f} cm in y",
              flush=True)

    def _poses(self):
        tr = _arr(self.view.get_transforms())
        return {b: (tr[i, :3] - self.origin, quat_to_matrix(tr[i, 3:7]), tr[i, 3:7]) for b, i in self.idx.items()}

    def physics_step(self) -> None:
        self.n += 1
        P = self._poses()
        vel = _arr(self.view.get_velocities())
        rp, rR, _ = P["rake"]
        if self.rake_start is None:
            self.rake_start = rp.copy()
        self.caught = ring_on_tine(P["ring"][0], P["ring"][1], rp, rR)
        moved = float(np.linalg.norm((rp - self.rake_start)[:2]))
        t = self.n * self.dt
        if not self.rake_off and rake_past_edge(rake_world_points(rp, rR)):
            self.rake_off = True
            print(f"[totekit] the rake went off the shelf at {t:.2f} s", flush=True)
        if not self.dragged and self.caught and moved > DRAG_M:
            self.dragged = True
            print(f"[totekit] the rake was dragged {moved * 100:.1f} cm with the ring still on its tine, at {t:.2f} s",
                  flush=True)
        tp, tR, _ = P["tote"]
        vz = vel[self.idx["tote"], 2]
        if not self.tote_dropped and vz < -DROP_V:
            self.tote_dropped = True
            print(f"[totekit] the tote fell ({vz:.2f} m/s) at {t:.2f} s", flush=True)
        if not self.tote_floor and tote_corners(tp, tR)[:, 2].min() < TOP - 0.02:
            self.tote_floor = True
            print(f"[totekit] the tote went below the table top at {t:.2f} s", flush=True)

    def state(self) -> ToteState:
        P = self._poses()
        vel = _arr(self.view.get_velocities())
        tp, tR, tq = P["tote"]
        rp, rR, _ = P["rake"]
        pads, opened = {}, {}
        for arm in ARMS:
            asset = self.base.scene[arm]
            poses = _arr(asset.data.body_link_pose_w)[0]
            tips = [poses[i, :3] - self.origin for i in self.pad_ids[arm][:2]]
            g = poses[self.pad_ids[arm][2]]
            tcp = g[:3] - self.origin + quat_to_matrix(g[3:7])[:, 2] * TCP_OFFSET
            pads[arm] = np.array([*tips, tcp])
            fingers = _arr(asset.data.joint_pos)[0, self.finger_ids[arm]]
            opened[arm] = float(np.clip(fingers.mean() / FINGER_OPEN, 0.0, 1.0))
        corners = tote_corners(tp, tR)
        caught = ring_on_tine(P["ring"][0], P["ring"][1], rp, rR)
        moved = float(np.linalg.norm((rp - (self.rake_start if self.rake_start is not None else rp))[:2]))
        st = ToteState(pos=tp, quat_xyzw=tq, lin_vel=vel[0, :3], ang_vel=vel[0, 3:], low=corners.min(0),
                       high=corners.max(0), tote_R=tR, corners=corners, ring_pos=P["ring"][0], ring_caught=caught,
                       rake_pos=rp, rake_pts=rake_world_points(rp, rR), rake_moved=moved,
                       rake_vel=float(np.linalg.norm(vel[self.idx["rake"], :3])), rake_off=self.rake_off,
                       dragged=self.dragged, tote_dropped=self.tote_dropped, tote_floor=self.tote_floor, pads=pads,
                       open_frac=opened)
        if self.debug and self.n % 120 == 0:
            print(f"[totekit] t {self.n * self.dt:6.2f} tote {np.round(tp, 3)} low {(corners[:4, 2].min() - TOP) * 100:5.1f} cm "
                  f"ring {np.round(P['ring'][0], 3)} caught {caught} rake moved {moved * 100:.1f} cm "
                  f"off {self.rake_off} dragged {self.dragged} dropped {self.tote_dropped} open {opened}", flush=True)
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks ------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_totekit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, ToteRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._totekit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_totekit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._totekit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_totekit", False):
        return
    ensure_sim, reset = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None:
            _hook_physics(self._env)
        return out

    def _reset(self, scene, *, seed=None):
        self._ensure_sim(None)
        for name in _SPECS:
            _runtime(self._base, name).reset(seed)
        obs = reset(self, scene, seed=seed)
        for name in _SPECS:
            _runtime(self._base, name).clear_latches()  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._totekit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's closed-loop ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_totekit", False):
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

    cls.reset, cls.act, cls._totekit = _reset, _act, True


def _patch_demo(demo) -> None:
    """``Waypoint.yaw`` may be a 3x3 rotation matrix (the demo's level grips)."""
    if getattr(demo, "_totekit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._totekit = _grip_rotation, True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment,
            "yam_arena.scripted": _patch_scripted, "yam_arena.demo": _patch_demo}


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
