"""Oracle pilot: follows a pre-planned racing line through the known gate layout.

Reference/testing pilot only - it relies on ground-truth gate geometry, so it is
NOT competition-legal (no absolute coordinates are provided in the qualifier).
Useful as a fast upper-bound baseline to measure the vision pilot against.
"""

from .oracle_pilot import update_trajectory_control

__all__ = ["update_trajectory_control"]
