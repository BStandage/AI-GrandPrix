"""The sim's solver API dataclasses, for machines without the sim repo (the
Orin). Field-for-field the same as elodin-sim-aigp/solver/api.py so the
follower and the seeker import either."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class RCCommand:
    throttle: int = 1000
    roll: int = 1500
    pitch: int = 1500
    yaw: int = 1500
    arm: int = 1000
    aux2: int = 1500
    aux3: int = 1500
    aux4: int = 1500


@dataclass
class SensorUpdate:
    t: float
    tick: int
    world_pos: np.ndarray
    world_vel: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    gyro_fresh: bool = True
    accel_fresh: bool = True
    baro: float = 0.0
    baro_fresh: bool = False
    mag: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mag_fresh: bool = False
    frame_rgba: Optional[np.ndarray] = None
    frame_fresh: bool = False
    next_gate_index: int = 0
    last_gate_passed: int = -1
