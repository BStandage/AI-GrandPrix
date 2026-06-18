"""Shared filesystem locations, anchored to the src package root.

Every module that reads or writes recorded data goes through here so that paths
resolve to the same place regardless of which subpackage the caller lives in or
what the current working directory is. ``PKG_ROOT`` is the src/ directory
(the parent of this ``common`` package).
"""

import os

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Recorded flight datasets (sessions, characterize/sysid CSVs, cached gates).
# Gitignored; created on demand by writers.
DATASETS_DIR = os.path.join(PKG_ROOT, "datasets")

# Scratch image dumps from the detector debug tool.
DEBUG_DIR = os.path.join(PKG_ROOT, "debug")
