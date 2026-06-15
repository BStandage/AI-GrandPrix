"""
Vision-pilot PERCEPTION / ESTIMATION params (the "header").

There are deliberately NO control gains here. The vision pilot estimates a racing line and flies it
with the shared measured-dynamics follower (common.line_follower) - the SAME control as the oracle.
The flight-dynamics campaign already nailed those gains; we do not re-tune them per pilot. Everything
below is about turning camera detections into a gate line, not about how to fly.

vision_pilot.py does `from .config import *`, so every VIS_* name here is available there.
"""

# Camera geometry: single source of truth (ground truth, spec VADR-TS-002).
from common.camera import HALF_TAN_X as VIS_HALF_TAN_X, HALF_TAN_Y as VIS_HALF_TAN_Y, \
    UPTILT_RAD as VIS_CAM_UPTILT

# --- gate range estimate (the WEAK axis) ----------------------------------------------------
# Cross-track (lateral+vertical) comes from the image bearing and is reliable (~1 m in the data).
# RANGE is unreliable (PnP/pinhole off by tens of m, and weakly observable on a straight approach),
# so gate depth rides on DEAD RECKONING - seeded once from the pinhole range inside a trusted band,
# then only nudged gently toward it. These bound how much we ever trust a measured range.
VIS_ACQ_MIN_RANGE = 5.0      # m: trusted pinhole-range band for seeding/correcting depth (below this
VIS_ACQ_MAX_RANGE = 32.0     # m: the gate fills/leaves the frame; above it the blob is too small)
VIS_RANGE_GAIN = 0.15        # per-detection blend of the measured range toward the estimate (low =
                             # trust dead reckoning; the measurement only trims slow drift)
VIS_RANGE_OUTLIER = 12.0     # m: ignore a measured range this far from the estimate (a blow-up)
VIS_RANGE_BIAS = 1.6         # the pinhole range reads ~38% SHORT vs truth (measured on real logs),
                             # which placed gates too NEAR -> wrong lateral on the sideways-jog gate.
                             # Scale the measured range up to undo the bias before fusing.

# --- gate tracking / association ------------------------------------------------------------
# A tracked gate is STATIC in the local frame, so its estimate should barely move (only refine
# slowly). A fast-swinging estimate is spurious (detection noise + ~1-frame camera latency rotated
# by the current attitude while banking) and, fed into the follower, drove a roll PIO. So fuse
# gently AND hard-cap how far the estimate may move per update.
VIS_FUSE_ALPHA = 0.12        # how hard each detection pulls a tracked gate onto the camera bearing
                             # (cross-track correction). Lower = smoother (less PIO), slower to centre.
VIS_EST_MAX_STEP = 0.08      # m: max the gate estimate may move per detection. The static-gate prior
                             # that kills the oscillation - big per-frame jumps are rejected as noise.
VIS_ASSOC_ANG = 0.30         # rad (~17 deg): a detection within this bearing of a tracked gate's
                             # predicted direction is that SAME gate (temporal persistence / anti-flip)
VIS_KEEP_FRAC = 0.55         # keep tracking the current gate only while its blob is >= this fraction
                             # of the largest blob ahead. Below it, a clearly NEARER gate has appeared
                             # (the one to fly through next) - switch to it. Stops it skipping gate 2.
                             # (0.72 fixed gate 3 but made it switch off gate 1 too early - reverted.)
VIS_EL_BIAS_DEG = 0.0        # subtract a measured elevation bias (deg) from the image bearing if the
                             # drone flies consistently high/low once the PIO is gone. + = read higher
                             # (drone flies lower). Set from a ground-truth-broadcast run via vision_diag.
VIS_PASS_DIST = 2.5          # m: once the gate estimate is this close (or behind), it's passed -
                             # release it and promote the next gate to the target.

# --- racing line ----------------------------------------------------------------------------
VIS_MIN_GATE_GAP = 4.0       # m: only fold the NEXT gate into the line if its estimate is at least
                             # this far BEYOND the current gate. Gate depth is unreliable, so a far
                             # blob's range often collapses onto the current gate's depth - two line
                             # knots on top of each other make a degenerate spline that brakes the
                             # speed profile to ~0 and parks the drone short. This rejects that.
VIS_REBUILD_EVERY = 25       # rebuild the spline every N control ticks (not every tick): the gate
                             # estimate drifts slowly, so re-splining at 250 Hz just injects jitter.
# --- vertical: WORLD-elevation servo (range-independent AND pitch-compensated) ---------------
# A fixed image ROW is wrong: the row a gate sits at depends on the drone's PITCH (it flies ~17 deg
# nose-down, which drops a level gate to row ~0.83, not a fixed target) - so a row servo descends
# forever and flies under the gates. Instead servo the gate's elevation in the WORLD frame to ~0:
# world_dir = R @ image_bearing already folds in pitch, and an elevation is a DIRECTION so the bad
# gate range can't corrupt it. elev > 0 = gate above us -> climb; < 0 = below -> descend. Servoing to
# 0 flies us to the gate's altitude. desired_climb = VIS_KP_VE * (elev - VIS_TARGET_ELEV).
VIS_TARGET_ELEV = 0.04       # rad: servo the gate's measured world elevation to this. real_miss showed
                             # it clipping the TOP of gates 0-2 by ~0.1 m (UP0.8-0.9), so raise a bit
                             # more for clean clearance + margin. RAISE if it still clips the top, LOWER
                             # if it starts flying under / into the floor.
VIS_KP_VE = 14.0              # m/s of climb per rad of world-elevation error
VIS_VERT_ALPHA = 0.28         # smoothing on the gate elevation (held through detection dropouts)
VIS_APPROACH_DIST = 1.0      # m before the gate to aim through first
VIS_APPROACH_MIN_RANGE = 5.0 # only add approach point when not already close


# --- lateral: body-AZIMUTH servo (range-independent), mirror of the vertical ----------------
# Same reasoning as vertical: gate range is garbage, and lateral position = range x bearing, so on
# the one gate that jogs sideways (gate 2) the bad range threw the target 6 m off. Servo the gate's
# BEARING (body azimuth) directly instead - range-free. a_strafe = KP*azimuth - KD*v_right -> roll.
VIS_KP_LAT = 8.0             # m/s^2 of strafe accel per rad of gate body-azimuth
VIS_KD_LAT = 2.5             # m/s^2 per m/s of body-right velocity (damping - prevents weave)
VIS_AZ_ALPHA = 0.3           # smoothing on the gate azimuth (held through detection dropouts)
VIS_AZ_MAX = 0.25            # rad: cap the azimuth fed to the servo. As the drone nears/passes a gate
                             # it sweeps to the side and the bearing blows up (-41 deg seen); capping
                             # stops the servo over-strafing at the worst moment.
VIS_COMMIT_RANGE = 5.0       # m: ramp the strafe correction to zero over the last (PASS_DIST + this)
                             # metres and COMMIT straight through - you can't change your arrival
                             # point that late, and chasing the sweeping bearing only lurches it.

VIS_V_MAX = 5.0             # m/s target speed for the estimated line. Below the oracle's 13: depth
                             # is uncertain and the horizon is only the next 1-2 gates, so carry a bit
                             # less speed. (Raise toward 13 once the estimate proves solid.)
VIS_FLOOR_MARGIN = 1.0       # m extra floor-guard margin below the lowest ESTIMATED gate (the gate
                             # z estimate is noisier than the oracle's ground truth, so keep clearance).
