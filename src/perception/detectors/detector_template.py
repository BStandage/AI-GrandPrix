"""
Template for a new detector. Copy this file to detectors/<name>.py, keep the class that fits your
approach, implement its one method, and register it in __init__.py.

If you want a model that returns a segmentation mask, keep ExampleMaskDetector (rename it, e.g.
CNNMaskDetector or YoloMaskDetector).
If you want a model that returns the gate geometry (offset, area, distance), keep ExampleDirectDetector
(rename it too).
"""

import numpy as np

from perception.detector import MaskDetector, DirectDetector
from perception.gate_detection import GateDetection


class ExampleMaskDetector(MaskDetector):
    """Segments the gate pixels into a mask. The shared geometry derives position, size, and range."""

    def segment(self, image):
        # image is the camera frame: a 360 x 640 grid of pixels, each pixel 3 numbers
        # (blue, green, red) from 0-255.
        # Return a mask of the same height and width, gate pixels nonzero (e.g. 255), the rest 0.
        # Replace this placeholder with real segmentation (color threshold, network, depth, etc).
        return np.zeros(image.shape[:2], dtype=np.uint8)


class ExampleDirectDetector(DirectDetector):
    """Produces the gate's position directly, without a mask."""

    def detect(self, image):
        # image is the camera frame: a 360 x 640 grid of pixels, each pixel 3 numbers
        # (blue, green, red) from 0-255.
        # Return one GateDetection per visible gate, or [] if none. Fields:
        #   offset_x    left/right in the image, -1 (left edge) to +1 (right edge)
        #   offset_y    up/down in the image, -1 (top edge) to +1 (bottom edge)
        #   area        apparent size in pixels, larger is nearer
        #   distance_m  rough range to the gate, in meters
        # Example, a gate centered in the frame roughly 12 m out:
        #   return [GateDetection(offset_x=0.0, offset_y=0.0, area=5000.0, distance_m=12.0)]
        return []
