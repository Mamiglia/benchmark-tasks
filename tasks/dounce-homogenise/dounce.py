"""The Dounce's liquid and stroke book-keeping (pure Python, no simulator).

Lengths in metres, volumes in m^3 unless named ``_ml``, times in seconds. ``z`` is the pestle head's bottom face
above the bore's floor (the prismatic joint's position).

With a 0.05 mm fit the head acts as a piston: liquid passes it only through the clearance. While the head is
low the sample lies below it and above it in the annulus round the shaft; as the head rises the sample drains past
it to below. Once all of it is below the head (``z > z_air``) a further pull draws air in past the head, and the
detergent sample foams. Pulling faster than the cavitation speed (suction over one atmosphere across the head)
also draws bubbles. The model counts full strokes (head within ``BOTTOM`` of the floor, then at least ``RISE``
above it, then back to the floor), upstrokes that drew air, fast upstrokes, and the foam they leave.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

BORE_R = 0.0050  # 10 mm bore (13 mm OD tube)
SHAFT_R = 0.0030  # 6 mm glass shaft
HEAD_L = 0.012  # ground head length
TUBE_L = 0.082  # grinding tube, floor to the reservoir
BOTTOM = 0.003  # a stroke "reaches the bottom" with the head within 3 mm of the floor
RISE = 0.025  # ... and "is full" when the head then rises at least 25 mm above the floor
DRAG = 160.0  # N s/m on the head (8 N at 50 mm/s; README: how it was set and how to check it)
P_ATM = 101_325.0
V_CAV = P_ATM * math.pi * BORE_R**2 / DRAG  # m/s: suction reaches one atmosphere (about 50 mm/s)
FOAM_AIR_MM = 2.0  # foam left by one upstroke that drew air past the head
FOAM_CAV_MM = 0.5  # ... by one cavitating (too fast) upstroke
FOAM_OK_MM = 2.0  # the end photo passes with less foam than this
LIFT_MAX = 0.005  # mortar raised more than 5 mm off its seat: latched failure
TILT_MAX = 20.0  # deg: mortar tipped
STROKES = 10


def volume_for(seed) -> float:
    """The trial's sample volume, mL: 3.0 to 3.4, from the seed."""
    rng = np.random.default_rng(int(seed or 0) + 4241)
    return round(float(rng.uniform(3.0, 3.4)) * 20) / 20


def z_air(v_ml: float) -> float:
    """Head height above which the sample is all below the head and the next pull draws air."""
    return v_ml * 1e-6 / (math.pi * BORE_R**2)


def level(v_ml: float, z: float) -> float:
    """Height of the liquid's surface above the floor with the head's bottom at ``z`` (quasi-static)."""
    v = v_ml * 1e-6
    a, s = math.pi * BORE_R**2, math.pi * SHAFT_R**2
    if z >= z_air(v_ml):
        return v / a  # all below the head
    return (v + a * HEAD_L - s * (z + HEAD_L)) / (a - s)  # head submerged, rest above in the annulus


@dataclass
class Dounce:
    v_ml: float = 3.2
    t: float = 0.0
    strokes: int = 0
    air: int = 0  # upstrokes that drew air
    cav: int = 0  # upstrokes faster than V_CAV
    lifted: bool = False
    tipped: bool = False
    pulled: bool = False  # head pulled up to the reservoir (out of the bore)
    max_lift: float = 0.0
    max_tilt: float = 0.0
    max_z: float = 0.0
    z: float = 0.0
    _down: bool = True  # head has touched the floor since the last full rise
    _risen: bool = False  # ... and has since risen RISE
    _air_now: bool = False
    _cav_now: bool = False
    log: list = field(default_factory=list)

    def reset(self, v_ml: float) -> None:
        self.__init__(v_ml=v_ml)

    @property
    def foam_mm(self) -> float:
        return FOAM_AIR_MM * self.air + FOAM_CAV_MM * self.cav

    @property
    def failed(self) -> bool:
        return self.lifted or self.tipped or self.pulled

    def step(self, z: float, lift: float, tilt_deg: float, dt: float) -> None:
        """One control step: head height (relative to the mortar), the mortar's rise off its seat and its tilt.
        The head's speed is taken over the control step (what a video at 20 fps shows)."""
        self.t += dt
        zdot = (z - self.z) / dt if self.t > dt else 0.0
        self.z = z
        self.max_z = max(self.max_z, z)
        self.max_lift = max(self.max_lift, lift)
        self.max_tilt = max(self.max_tilt, tilt_deg)
        if lift > LIFT_MAX and not self.lifted:
            self.lifted = True
            self.log.append((self.t, "mortar lifted"))
        if tilt_deg > TILT_MAX and not self.tipped:
            self.tipped = True
            self.log.append((self.t, "mortar tipped"))
        if z > TUBE_L - HEAD_L and not self.pulled:
            self.pulled = True
            self.log.append((self.t, "pestle pulled out of the bore"))
        if z <= BOTTOM:
            if self._risen:
                self.strokes += 1
                self.log.append((self.t, f"stroke {self.strokes}"))
            self._down, self._risen = True, False
            self._air_now = self._cav_now = False
        if self._down and z >= RISE:
            self._risen = True
        if zdot > 0:  # upstroke: each counts at most once for air and once for cavitation
            if z > z_air(self.v_ml) and not self._air_now:
                self._air_now = True
                self.air += 1
                self.log.append((self.t, "air drawn in"))
            if zdot > V_CAV and not self._cav_now:
                self._cav_now = True
                self.cav += 1
                self.log.append((self.t, f"cavitation at {zdot * 1000:.0f} mm/s"))


def score(d: Dounce) -> float:
    """0 if the mortar was ever lifted > 5 mm or tipped > 20 deg, or the head was pulled out of the bore.
    Otherwise 0.08 per full stroke (up to 10), plus 0.2 if the foam is under 2 mm, minus 0.05 per upstroke that drew
    air past the head; floored at 0."""
    if d.failed:
        return 0.0
    s = 0.08 * min(d.strokes, STROKES) + (0.2 if d.foam_mm < FOAM_OK_MM else 0.0) - 0.05 * d.air
    return round(max(0.0, min(1.0, s)), 4)
