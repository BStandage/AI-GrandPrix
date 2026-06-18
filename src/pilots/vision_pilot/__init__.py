"""Vision pilot: flies from the forward camera alone, with no ground-truth gates.

This is the primary competition pilot and the main area of ongoing development.
"""

from .vision_pilot import update_vision_control

__all__ = ["update_vision_control"]
