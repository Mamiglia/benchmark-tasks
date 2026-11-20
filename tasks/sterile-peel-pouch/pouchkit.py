"""pouchkit: a sealed peel pouch holding dressing forceps, the block it lies on and a sterile field drape, for a
yam_arena task.

yam_arena task files only take single primitives. This module adds one object, :class:`PouchRig`, spawned as a fixed
(``BASE``) Arena asset holding:

- ``rest`` and ``drape``: static colliders: a 3 cm block the pouch lies flat on (its chevron end flush with the
  block's edge, so the lower ply's tab can hang down) and a blue sterile field drape lying flat on the table.
- ``paper_tab``, ``paper_0`` .. ``paper_{N-1}`` and the same for ``film``: the pouch's two plies, each a chain of rigid
  strips across the pouch's width joined by hinges (revolute joints about the width axis), so each ply bends along its
  length like the real sheet. The tab is the unsealed flap above the chevron, turned out.
- The geometry below is written for the pouch upright (chevron up, plies separated along y, as if it stood on a 6 mm
  base); :func:`flat` turns that frame 90 degrees about x onto the rest, so the pouch lies flat, paper up.
- seal: at every strip boundary (the chevron at the top, the bottom seal at the end and every line between) the two
  plies are joined by two point joints, one in each side seal. Each joint breaks (PhysX ``breakForce``) when the pull
  on it passes ``SEAL_BREAK_N``: pulling the two tabs apart parts the seal one boundary at a time, at the real peel
  force. Paper and film each carry half of the side and bottom seams, so the pocket is closed until it is peeled.
- ``forceps``: one rigid body, stainless dressing forceps (two blades joined at the spring end) inside the pocket.

Before every physics step (240 Hz) the kit latches: a finger pad or tool point near the forceps, or near the drape;
a pouch strip touching the drape. ``object_states`` reports a :class:`PouchState`.

Importing the module also hooks ``yam_arena.scripted`` so that ``yam_scripted`` plays the task file's ``controller``
(see ``peeldemo.py``). yam_arena itself is not modified. Pure Python at import; Isaac Sim is imported lazily.
Known limit: broken seal joints are not restored by a reset, so run one episode per process (inspect-robots does).
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
# Pouch: 3.5 x 9 in (89 x 229 mm) heat-seal peel pouch, medical paper (left, +y) and blue PET/PP film (right, -y),
# lying flat on the rest, paper up. 209 mm sealed body in N strips; a 20 mm unsealed tab beyond the chevron on each
# ply, turned out (paper tab up, film tab down over the rest's edge). Written in the upright design frame (see flat()).
POUCH_X = 0.24  # centre of the pouch's width
POUCH_W = 0.089
N = 11
STRIP_L = 0.019  # 11 x 19 mm = 209 mm sealed body
TAB_L = 0.020
PLY_T = 0.0015  # collider thickness of a ply (the real sheet is 0.1 mm; see fidelity)
GAP = 0.006  # pocket between the plies' inner faces (the forceps is 2.5 mm thick)
SEAM_X = POUCH_W / 2 - 0.004  # side seal joints 4 mm in from each side edge
SEAM_W = 0.003  # side and bottom seam lips
PLY_MASS = 0.0015  # per strip (the real ply is 0.15 g per strip; raised for solver stability, see fidelity)
SEAL_BREAK_N = 1.6  # per joint, two per boundary: 3.2 N to part a boundary (README: peel force)
HINGE_DAMPING = 2e-4  # N m s/rad: the sheet's internal friction on each hinge
TAB_STIFFNESS = 0.02  # N m/rad: the crease that keeps a turned-out tab level (it holds the tab's own weight)

# Upright design frame: the pouch as if standing chevron up on a 6 mm base.
POUCH_BOTTOM = TOP + 0.006
Z_SEAL_TOP = POUCH_BOTTOM + N * STRIP_L  # the chevron (top seal boundary)

# Rest: a 40 x 240 x 30 mm block (wood or PLA) on the table; the pouch lies flat on it, film down, chevron toward the
# right arm (-y) at y CHEVRON_Y, its body along +y. The block's -y face is 2 mm in from the chevron, so the film tab
# hangs free over the edge. Narrower than the pouch (89 mm), so the pouch's side edges overhang it.
REST_SIZE = (0.040, 0.240, 0.030)
REST_TOP = TOP + REST_SIZE[2]
CHEVRON_Y = 0.0
REST_C = (POUCH_X, CHEVRON_Y + 0.002 + REST_SIZE[1] / 2, TOP + REST_SIZE[2] / 2)
FLAT_R = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])  # 90 deg about x: upright -z (body) -> +y, +y (paper) -> +z
FLAT_Q = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # xyzw
PIVOT = np.array([POUCH_X, 0.0, Z_SEAL_TOP])  # the chevron's mid-line, upright frame
FLAT_PIVOT = np.array([POUCH_X, CHEVRON_Y, REST_TOP + GAP / 2 + PLY_T])  # ... lying flat: film's outer face on the rest


def flat(p) -> np.ndarray:
    """A point of the upright design frame, with the pouch lying flat on the rest."""
    return FLAT_PIVOT + FLAT_R @ (np.asarray(p, float) - PIVOT)


# Drape: 18 x 26 in (457 x 660 mm) sterile poly-lined towel drape, blue, flat on the table in front of the rest, its
# long side across (y). The outer 2.5 cm of a sterile field counts as non-sterile (AORN): the target is the inner area.
DRAPE_C = (0.52, 0.0)
DRAPE_HALF = (0.2285, 0.330)  # x, y
DRAPE_T = 0.001
DRAPE_TOP = TOP + DRAPE_T
BORDER = 0.025

# Forceps: 14 cm (5.5 in) stainless dressing forceps, 30 g: two 7 mm blades, 2.5 mm thick, joined at the spring end.
FORCEPS_L, BLADE_W, BLADE_T, BLADE_DX = 0.140, 0.007, 0.0025, 0.0055
FORCEPS_MASS = 0.030

# Latches.
TOUCH_M = 0.008  # a finger pad tip or tool point within 8 mm of the forceps: touched
DRAPE_TOUCH_M = 0.010  # a pad tip or tool point within 1 cm of the drape's top, over the drape: touched
POUCH_DRAPE_M = 0.002  # a pouch strip corner within 2 mm of the drape top, over the drape: touched
PARTED_M = 0.004  # a seal joint whose two anchors are 4 mm apart has broken


@dataclass(frozen=True)
class PouchRig(Box):
    """The whole rig. ``pos``/``size`` only describe a bounding box for yam_arena's checks."""

    def _check(self) -> None:
        super()._check()
        if self.mass is not None:
            raise ValueError(f"{self.name}: a PouchRig is a fixed asset (mass None); its bodies carry their own")


# -- geometry ----------------------------------------------------------------------------------------------------------

PLIES = ("paper", "film")
SIDE = {"paper": 1.0, "film": -1.0}
BODIES = tuple([f"{p}_tab" for p in PLIES] + [f"{p}_{k}" for p in PLIES for k in range(N)] + ["forceps"])


def strip_center(ply: str, k: int) -> np.ndarray:
    s = SIDE[ply]
    return np.array([POUCH_X, s * (GAP / 2 + PLY_T / 2), Z_SEAL_TOP - (k + 0.5) * STRIP_L])


def tab_center(ply: str) -> np.ndarray:
    """The tab lies level, turned out from the top of its ply."""
    s = SIDE[ply]
    return np.array([POUCH_X, s * (GAP / 2 + PLY_T + TAB_L / 2), Z_SEAL_TOP + PLY_T / 2])


STRIP_SIZE = (POUCH_W, PLY_T, STRIP_L)
TAB_SIZE = (POUCH_W, TAB_L, PLY_T)


def strip_parts(ply: str, k: int):
    """(centre, size) of a strip's colliders in its body frame: the sheet, half of each side seam and, on the last
    strip, half of the bottom seam (lips toward the other ply)."""
    s = SIDE[ply]
    lip_y = -s * (PLY_T / 2 + GAP / 4 - 0.00025)
    lip_h = GAP / 2 - 0.0005
    parts = [((0, 0, 0), STRIP_SIZE)]
    for sx in (-1, 1):
        parts.append(((sx * (POUCH_W / 2 - SEAM_W / 2), lip_y, 0), (SEAM_W, lip_h, STRIP_L)))
    if k == N - 1:
        parts.append(((0, lip_y, -STRIP_L / 2 + SEAM_W / 2), (POUCH_W - 2 * SEAM_W, lip_h, SEAM_W)))
    return parts


def seal_anchors(b: int):
    """For seal boundary b (0 = chevron, N = bottom seal): [(paper body, paper local, film body, film local)] for the
    two side joints. Both locals map to the paper ply's inner face at the boundary."""
    k = min(b, N - 1)
    z = STRIP_L / 2 if b < N else -STRIP_L / 2
    out = []
    for sx in (-1, 1):
        x = sx * SEAM_X
        out.append((f"paper_{k}", (x, -PLY_T / 2, z), f"film_{k}", (x, GAP + PLY_T / 2, z)))
    return out


def forceps_center() -> np.ndarray:
    return np.array([POUCH_X, 0.0, POUCH_BOTTOM + SEAM_W + 0.0008 + FORCEPS_L / 2])


def forceps_parts():
    """(centre, size) in the forceps' body frame: two blades and the spring end that joins them."""
    return [((-BLADE_DX, 0, 0), (BLADE_W, BLADE_T, FORCEPS_L)), ((BLADE_DX, 0, 0), (BLADE_W, BLADE_T, FORCEPS_L)),
            ((0, 0, FORCEPS_L / 2 - 0.006), (2 * BLADE_DX + BLADE_W, BLADE_T, 0.012))]


def box_corners(pos, R, centre, size) -> np.ndarray:
    c = np.array([(sx * size[0] / 2, sy * size[1] / 2, sz * size[2] / 2)
                  for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) + np.asarray(centre)
    return np.asarray(pos) + c @ np.asarray(R).T


def forceps_corners(pos, R) -> np.ndarray:
    return np.concatenate([box_corners(pos, R, c, s) for c, s in forceps_parts()[:2]])


def gap_to_box(p, pos, R, centre, size) -> float:
    q = np.asarray(R).T @ (np.asarray(p, float) - np.asarray(pos)) - np.asarray(centre)
    return float(np.linalg.norm(np.maximum(np.abs(q) - np.asarray(size) / 2, 0.0)))


def gap_to_forceps(p, pos, R) -> float:
    return min(gap_to_box(p, pos, R, c, s) for c, s in forceps_parts())


def over_drape(p, margin: float = 0.0) -> bool:
    """Point's (x, y) inside the drape, shrunk by ``margin`` on every side."""
    return abs(p[0] - DRAPE_C[0]) <= DRAPE_HALF[0] - margin and abs(p[1] - DRAPE_C[1]) <= DRAPE_HALF[1] - margin


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def seals_parted(P) -> list[bool]:
    """Per boundary 0..N: both of its side joints parted (anchors more than PARTED_M apart). P: body -> (pos, R)."""
    out = []
    for b in range(N + 1):
        ok = True
        for pb, pl, fb, fl in seal_anchors(b):
            a = P[pb][0] + P[pb][1] @ np.asarray(pl)
            c = P[fb][0] + P[fb][1] @ np.asarray(fl)
            ok &= float(np.linalg.norm(a - c)) > PARTED_M
        out.append(bool(ok))
    return out


def peeled_mm(parted) -> float:
    """Length of seal opened from the chevron down: boundaries parted in a row from the top."""
    n = 0
    for p in parted:
        if not p:
            break
        n += 1
    return 0.0 if n == 0 else min(n - 1, N) * STRIP_L * 1000


def pouch_corners(P) -> np.ndarray:
    pts = [box_corners(*P[f"{p}_tab"][:2], (0, 0, 0), TAB_SIZE) for p in PLIES]
    pts += [box_corners(*P[f"{p}_{k}"][:2], (0, 0, 0), STRIP_SIZE) for p in PLIES for k in range(N)]
    return np.concatenate(pts)


def pouch_touches_drape(pts) -> bool:
    return any(over_drape(p) and p[2] < DRAPE_TOP + POUCH_DRAPE_M for p in pts)


def pads_touch_drape(pads) -> bool:
    return any(over_drape(p) and p[2] < DRAPE_TOP + DRAPE_TOUCH_M for pp in pads.values() for p in pp)


@dataclass(frozen=True)
class PouchState(ObjectState):
    """The rig after a control step (env frame). ``pos``/``quat_xyzw``/velocities are the forceps'."""

    forceps_R: np.ndarray = None
    forceps_pts: np.ndarray = None  # (16, 3) blade corners
    parted: tuple = ()  # per seal boundary, 0 (chevron) .. N (bottom seal)
    peeled_mm: float = 0.0
    pouch_pts: np.ndarray = None
    touched_forceps: bool = False  # latched
    touched_drape: bool = False  # latched: a gripper
    pouch_on_drape: bool = False  # latched
    pads: dict = field(default_factory=dict)
    open_frac: dict = field(default_factory=dict)


# -- spawning (Isaac Sim only) ----------------------------------------------------------------------------------

_SPECS: dict[str, PouchRig] = {}
_PATHS: dict[str, str] = {}
COLORS = {"rest": (0.85, 0.85, 0.82), "drape": (0.16, 0.42, 0.78), "paper": (0.95, 0.95, 0.92),
          "film": (0.55, 0.75, 0.92), "steel": (0.72, 0.73, 0.75), "seam": (0.86, 0.86, 0.80)}
FRICTION = {"rest": 0.4, "drape": 0.5, "paper": 0.9, "film": 0.9, "steel": 0.35}


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

    def cube(parent, name, centre, size, mat, col):
        c = UsdGeom.Cube.Define(stage, f"{parent}/{name}")
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        c.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        prim = c.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        pc = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        pc.CreateContactOffsetAttr(0.001)
        pc.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material(mat), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(col), stage=stage)

    def body(name, pos, mass):
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=dbl).Set(Gf.Vec3d(*map(float, flat(pos))))
        xf.AddOrientOp(precision=dbl).Set(Gf.Quatd(FLAT_Q[3], *FLAT_Q[:3]))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(8)
        rb.CreateAngularDampingAttr(0.5)
        rb.CreateLinearDampingAttr(0.1)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        return path

    static = f"{prim_path}/static"
    UsdGeom.Xform.Define(stage, static)
    cube(static, "rest", REST_C, REST_SIZE, "rest", "rest")
    cube(static, "drape", (*DRAPE_C, TOP + DRAPE_T / 2), (2 * DRAPE_HALF[0], 2 * DRAPE_HALF[1], DRAPE_T), "drape", "drape")

    paths = {}
    for ply in PLIES:
        paths[f"{ply}_tab"] = body(f"{ply}_tab", tab_center(ply), PLY_MASS)
        cube(paths[f"{ply}_tab"], "sheet", (0, 0, 0), TAB_SIZE, ply, ply)
        for k in range(N):
            b = body(f"{ply}_{k}", strip_center(ply, k), PLY_MASS)
            paths[f"{ply}_{k}"] = b
            for i, (c, s) in enumerate(strip_parts(ply, k)):
                cube(b, f"part{i}", c, s, ply, ply if i == 0 else "seam")
    fp = body("forceps", forceps_center(), FORCEPS_MASS)
    for i, (c, s) in enumerate(forceps_parts()):
        cube(fp, f"part{i}", c, s, "steel", "steel")

    def joint(kind, name, b0, b1, p0, p1, brk=None, stiffness=0.0):
        j = (UsdPhysics.RevoluteJoint if kind == "hinge" else UsdPhysics.SphericalJoint).Define(
            stage, f"{prim_path}/joints/{name}")
        j.CreateBody0Rel().SetTargets([paths[b0]])
        j.CreateBody1Rel().SetTargets([paths[b1]])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, p0)))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*map(float, p1)))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr("X")
        j.CreateCollisionEnabledAttr(False)
        if brk is not None:
            j.CreateBreakForceAttr(float(brk))
        if kind == "hinge":
            d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
            d.CreateTypeAttr("force")
            d.CreateStiffnessAttr(stiffness * math.pi / 180.0)  # USD angular drive gains are per degree
            d.CreateDampingAttr(HINGE_DAMPING * math.pi / 180.0)
            d.CreateTargetPositionAttr(0.0)

    UsdGeom.Xform.Define(stage, f"{prim_path}/joints")
    for ply in PLIES:
        s = SIDE[ply]
        # tab: hinged at the top of strip 0, on the ply's outer face
        joint("hinge", f"{ply}_tabhinge", f"{ply}_0", f"{ply}_tab", (0, s * PLY_T / 2, STRIP_L / 2),
              (0, -s * TAB_L / 2, -PLY_T / 2), stiffness=TAB_STIFFNESS)
        for k in range(N - 1):
            joint("hinge", f"{ply}_hinge{k}", f"{ply}_{k}", f"{ply}_{k + 1}", (0, 0, -STRIP_L / 2),
                  (0, 0, STRIP_L / 2))
    for b in range(N + 1):
        for i, (pb, pl, fb, fl) in enumerate(seal_anchors(b)):
            joint("seal", f"seal{b}_{i}", pb, fb, pl, fl, brk=SEAL_BREAK_N)
    return root.GetPrim()


def _arena_object(spec: PouchRig):
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


def start_transforms() -> np.ndarray:
    tr = np.zeros((len(BODIES), 7))
    tr[:, 3:] = FLAT_Q
    for i, b in enumerate(BODIES):
        if b == "forceps":
            tr[i, :3] = forceps_center()
        elif b.endswith("_tab"):
            tr[i, :3] = tab_center(b[:-4])
        else:
            ply, k = b.split("_")
            tr[i, :3] = strip_center(ply, int(k))
        tr[i, :3] = flat(tr[i, :3])
    return tr


class _Runtime:
    """PhysX view and latches for one rig (env 0)."""

    def __init__(self, base, spec: PouchRig):
        from isaaclab_physx.physics import PhysxManager

        self.spec, self.base = spec, base
        root = _PATHS[spec.name]
        view = PhysxManager.get_physics_sim_view()
        self.view = view.create_rigid_body_view([f"{root}/{b}" for b in BODIES])
        self.idx = {b: i for i, b in enumerate(BODIES)}
        self.dt = float(base.physics_dt)
        self.origin = _arr(base.scene.env_origins)[0]
        self.debug = bool(os.environ.get("POUCH_DEBUG"))
        self.pad_ids = {arm: [base.scene[arm].find_bodies(b)[0][0] for b in ("tip_left", "tip_right", "gripper")]
                        for arm in ARMS}
        self.finger_ids = {arm: base.scene[arm].find_joints(["joint7", "joint8"])[0] for arm in ARMS}
        self.clear_latches()

    def clear_latches(self) -> None:
        self.touched_forceps = self.touched_drape = self.pouch_on_drape = False
        self.n = 0
        self.last_peel = -1.0

    def reset(self) -> None:
        tr = start_transforms()
        tr[:, :3] += self.origin
        cur = self.view.get_transforms()
        ids = np.arange(len(BODIES), dtype=np.int32)
        self.view.set_transforms(_like(cur, tr), _like(cur, ids, "int32"))
        self.view.set_velocities(_like(cur, np.zeros((len(BODIES), 6))), _like(cur, ids, "int32"))
        self.clear_latches()

    def _poses(self):
        tr = _arr(self.view.get_transforms())
        return {b: (tr[i, :3] - self.origin, quat_to_matrix(tr[i, 3:7]), tr[i, 3:7]) for b, i in self.idx.items()}

    def _pads(self):
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
        return pads, opened

    def physics_step(self) -> None:
        self.n += 1
        if self.n % 4:  # 60 Hz is enough for the latches
            return
        P = self._poses()
        pads, _ = self._pads()
        t = self.n * self.dt
        fp, fR, _ = P["forceps"]
        if not self.touched_forceps:
            d = min(gap_to_forceps(p, fp, fR) for pp in pads.values() for p in pp)
            if d < TOUCH_M:
                self.touched_forceps = True
                print(f"[pouchkit] a gripper touched the forceps ({d * 1000:.1f} mm) at {t:.2f} s", flush=True)
        if not self.touched_drape and pads_touch_drape(pads):
            self.touched_drape = True
            print(f"[pouchkit] a gripper touched the drape at {t:.2f} s", flush=True)
        if not self.pouch_on_drape and pouch_touches_drape(pouch_corners(P)):
            self.pouch_on_drape = True
            print(f"[pouchkit] the pouch touched the drape at {t:.2f} s", flush=True)
        if self.n % 48 == 0:
            pm = peeled_mm(seals_parted(P))
            if pm != self.last_peel:
                self.last_peel = pm
                print(f"[pouchkit] t {t:6.2f} seal opened {pm:.0f} mm", flush=True)

    def state(self) -> PouchState:
        P = self._poses()
        vel = _arr(self.view.get_velocities())
        fp, fR, fq = P["forceps"]
        pads, opened = self._pads()
        parted = seals_parted(P)
        pts = forceps_corners(fp, fR)
        pouch = pouch_corners(P)
        i = self.idx["forceps"]
        st = PouchState(pos=fp, quat_xyzw=fq, lin_vel=vel[i, :3], ang_vel=vel[i, 3:], low=pts.min(0), high=pts.max(0),
                        forceps_R=fR, forceps_pts=pts, parted=tuple(parted), peeled_mm=peeled_mm(parted),
                        pouch_pts=pouch, touched_forceps=self.touched_forceps, touched_drape=self.touched_drape,
                        pouch_on_drape=self.pouch_on_drape, pads=pads, open_frac=opened)
        if self.debug and self.n % 120 < 24:
            print(f"[pouchkit] t {self.n * self.dt:6.2f} forceps {np.round(fp, 3)} low {(pts[:, 2].min() - DRAPE_TOP) * 1000:6.1f} mm "
                  f"parted {''.join('x' if p else '.' for p in parted)} pouch low {(pouch[:, 2].min() - TOP) * 1000:.0f} mm "
                  f"tabs {np.round(P['paper_tab'][0], 3)} {np.round(P['film_tab'][0], 3)} "
                  f"pads L {np.round(pads['left_robot'][2], 3)} R {np.round(pads['right_robot'][2], 3)}", flush=True)
        return st


_RT: dict[str, _Runtime] = {}


def _runtime(base, name: str) -> _Runtime:
    if name not in _RT:
        _RT[name] = _Runtime(base, _SPECS[name])
    return _RT[name]


# -- hooks ------------------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_pouchkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def _task_object(spec):
        if isinstance(spec, PouchRig):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def _object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _runtime(environment.unwrapped, n).state() for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states, scene._pouchkit = _task_object, _object_states, True


def _hook_physics(environment) -> None:
    base = environment.unwrapped
    isc = base.scene
    if getattr(isc, "_pouchkit", False):
        return
    write = isc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for name in _SPECS:
            _runtime(base, name).physics_step()
        return write(*a, **k)

    isc.write_data_to_sim, isc._pouchkit = write_data_to_sim, True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_pouchkit", False):
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
            _runtime(self._base, name).reset()
        obs = reset(self, scene, seed=seed)
        for name in _SPECS:
            _runtime(self._base, name).clear_latches()  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._pouchkit = _ensure_sim, _reset, True


def _patch_scripted(module) -> None:
    """``yam_scripted`` plays the task file's ``controller`` when it has one."""
    cls = module.YamScriptedPolicy
    if getattr(cls, "_pouchkit", False):
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

    cls.reset, cls.act, cls._pouchkit = _reset, _act, True


def _patch_demo(demo) -> None:
    """``Waypoint.yaw`` may be a 3x3 rotation matrix (the demo's side grips)."""
    if getattr(demo, "_pouchkit", False):
        return
    grip_rotation = demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else grip_rotation(yaw)

    demo.grip_rotation, demo._pouchkit = _grip_rotation, True


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
