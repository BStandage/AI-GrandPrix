"""Steady pilot: fighter-pilot bearing servo. Course-adaptive, reactive, no hardcoded line.

Rips the bank over to centre the current gate (the only reliable perception signal), servos thrust to
its elevation, cruises forward but eases off when off-axis so sharp turns are banked not charged.
Rate interface with yaw_rate=0 (no absolute-heading dependence). Works on any map.
"""

from .steady_pilot import update_steady_control

__all__ = ["update_steady_control"]
