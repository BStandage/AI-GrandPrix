"""Path shim so tests import the loader (src package) and the extractor
script (loaded from file — scripts/ is not a package)."""
import importlib.util
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)


def load_extractor():
    p = os.path.join(REPO, "scripts", "extract_course_map.py")
    spec = importlib.util.spec_from_file_location("overhead_extractor", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
