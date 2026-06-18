"""
Detector registry. Each gate detector has a unique file in this folder. Register it with one line below.

Select which one flies with the GATE_DETECTOR environment variable. When it's unset, the detector
named by DEFAULT_DETECTOR below ('hsv_classic') is used:

    GATE_DETECTOR=hsv_classic python main.py      # the classic-CV reference (default)
    GATE_DETECTOR=my_cnn      python main.py      # a registered custom detector

"""

import importlib
import os

DEFAULT_DETECTOR = "hsv_classic"

# name -> "module path:ClassName". Add one line here to register a new detector.
REGISTRY = {
    "hsv_classic": "perception.detectors.hsv_classic:HsvClassic",
    # "my_cnn":     "perception.detectors.my_cnn:MyCNN",        # see detector_template.py
}


def active_detector():
    """Instantiate the detector chosen by $GATE_DETECTOR"""

    # Resolves which detector to use.
    # The GATE_DETECTOR env var, or the default when it isn't set
    name = os.environ.get("GATE_DETECTOR", DEFAULT_DETECTOR)

    # Raise an error if the detector does not exist in the registry above ("REGISTRY")
    if name not in REGISTRY:
        raise KeyError(f"unknown GATE_DETECTOR={name!r}; registered: {sorted(REGISTRY)}")

    # The registry value is "module path:ClassName". Import that module and get the class
    module_path, class_name = REGISTRY[name].split(":")
    cls = getattr(importlib.import_module(module_path), class_name)

    # create one instance and hand it back to the caller (vision_rx).
    # from here on, every camera frame is run through this selected detector.
    return cls()
