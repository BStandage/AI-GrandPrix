"""
The detector interface. A detector turns one camera image into a list of GateDetection (defined in
gate_detection.py). 

Individual perception models, which we call detectors, live in their own files under detectors/.

Two base classes cover the two ways a detector can produce its output:

  MaskDetector         for detectors that segment the gate pixels into a mask. The subclass implements
                       segment(image), and the base runs mask_to_detections() to derive the
                       GateDetection list. Simpler, since the geometry is shared.

  DirectDetector       for detectors that regress the gate's position directly (no mask). The subclass
                       implements detect(image) and returns the GateDetection list itself.

process(image) is the entry point the runtime calls. The two base classes define it, not the
subclass. It runs the subclass's segment() or detect(), validates the result, and raises
DetectorError if the result breaks the contract. A bad detector then fails right here with a clear
message instead of feeding bad data into the flight code.

Detectors are registered in detectors/__init__.py and chosen at runtime with the GATE_DETECTOR
environment variable.
"""

from abc import ABC, abstractmethod

import numpy as np

from perception.gate_detection import GateDetection, mask_to_detections


class DetectorError(Exception):
    """A detector's output did not match the contract. The message states the specific violation."""


def _check_mask(mask, img_shape):
    """Validate a mask from segment(): a 2-D uint8 array the same height and width as the image."""
    if mask is None:
        raise DetectorError("segment() returned None. Return a uint8 mask with gate pixels nonzero.")
    if not isinstance(mask, np.ndarray):
        raise DetectorError(f"segment() must return a numpy array, got {type(mask).__name__}.")
    if mask.ndim != 2:
        raise DetectorError(f"mask must be 2-D (height x width), got {mask.ndim} dimensions.")
    if mask.shape != tuple(img_shape[:2]):
        raise DetectorError(
            f"mask is {tuple(mask.shape)} but the image is {tuple(img_shape[:2])}. They must match.")
    if mask.dtype != np.uint8:
        raise DetectorError(f"mask dtype is {mask.dtype}. Must be uint8 (0 = background, nonzero = gate).")


def _check_detections(dets):
    """Validate a return value from detect(): a list of GateDetection, each with in-range offsets."""
    if dets is None:
        raise DetectorError("detect() returned None. Return a list of GateDetection, or [] if no gate.")
    try:
        dets = list(dets)
    except TypeError:
        raise DetectorError(f"detect() must return a list, got {type(dets).__name__}.")
    for d in dets:
        if not isinstance(d, GateDetection):
            raise DetectorError(f"each item must be a GateDetection, got {type(d).__name__}.")
        if not (-1.0 <= d.offset_x <= 1.0 and -1.0 <= d.offset_y <= 1.0):
            raise DetectorError(
                f"offset ({d.offset_x:.3f}, {d.offset_y:.3f}) out of range. Both must be in [-1, 1].")
    return dets


class GateDetector(ABC):
    """Base for every detector. The framework calls process(image) and expects a GateDetection list."""

    @abstractmethod
    def process(self, image):
        ...


class MaskDetector(GateDetector):
    """Base for detectors that segment the gate into a mask; the shared geometry derives the rest.
    Subclasses implement segment()."""

    @abstractmethod
    def segment(self, image):
        """Return a uint8 mask the same height and width as image: gate pixels nonzero, background 0."""

    def process(self, image):
        mask = self.segment(image)
        _check_mask(mask, image.shape)
        return mask_to_detections(mask, image.shape)


class DirectDetector(GateDetector):
    """Base for detectors that produce the gate's position directly, without a mask. Subclasses
    implement detect()."""

    @abstractmethod
    def detect(self, image):
        """Return a list of GateDetection (empty list if no gate is seen)."""

    def process(self, image):
        return _check_detections(self.detect(image))
