BACKUP 2026-07-29 ~16:00 - "reaches gate 10" state, saved before the post-gate-2 stability regrind.

What this is: the complete working tape state at the moment we decided to stop pushing the
frontier (gate 11) and instead fix the unstable crossings after gate 2.

State: lock {upto: 10, t: 34.0}. Banked bytes carry gates 0-10 (era gate-10 approach,
gate-9 flare, gate-8 byte-win). Suffix = 10->11 floor descent with +0.7 right aim and the
3 m gate-11 exit pin (NEVER FLOWN - no run since 143152 reached the suffix).

Measured stability on these exact bytes (50 flights, 2026-07-29 14:00-15:40, impact-impulse
ledger): g1 ~88% (mixed edges + relaunch degradation), g3 ~88% (mixed), g4 ~94% (right edge),
g5 ~88% (top edge, 5/5 pushed down), g6 ~82% (top edge, 6/6), g8 ~89% (left side),
g9 ~50% (LEFT side, 7/7 pushed right - frame edge and/or pole), g10 ~30% (bottom-right-ish).
End-to-end completion ~10-25%.

To restore: copy all files back into src/pilots/ace_pilot/ (config.py included - caps matter).
