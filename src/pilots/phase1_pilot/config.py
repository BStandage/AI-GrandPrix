# Phase 1 aggressive racing config
# Based on sysid campaign results (datasets/sysid_merged_*/SYSID_REPORT.md):
#   roll/pitch omega_max = 29 rad/s
#   roll/pitch alpha_max = 902 rad/s²
#   lag to 63% = 96ms
#   terminal fwd speed = 9.38 m/s
#   drag k = 0.0343 (a = k·v²)

# Racing line shape
P1_V_MAX = 13.0          # physics caps at 9.38 anyway
P1_APEX_MAX = 0.0        # thread gate centres — flyable hole is only ±0.61 m, apex-cut clips the post
P1_A_LAT = 12.0          # lateral accel design point — matches ~52° bank

# Attitude limits — unlocked to airframe capability
P1_BRAKE_PITCH = -0.70   # was -0.35 (~20°) → ~40° nose-up braking
P1_ACCEL_PITCH = 1.05    # was 0.65 (~37°) → ~60° nose-down acceleration
P1_MAX_STRAFE = 0.85     # unchanged

# Slew rates — THE critical bottleneck
# At 2.5/4.0 rad/s the attitude takes 300-400ms to settle
# Airframe physical lag is 96ms at 902 rad/s² — we're throwing away 3-4x authority
P1_PITCH_SLEW = 15.0     # was 2.5
P1_ROLL_SLEW = 15.0      # was 4.0

# Yaw — keep conservative, yaw lag is 153ms and omega_max only 19 rad/s
P1_YAW_KP = 0.7
P1_MAX_YAW_RATE = 2.0

# Vertical
P1_KP_H = 3.0
P1_VFF = 0.5             # was 0.8 — less slope feed-forward so it arrives at the descent bottom higher/slower (No-Go: 4.76 m/s plunge needs 2.48 m to arrest, only ~2 m there)
P1_VERT_BIAS = 0.3

# --- Round-1 course-specific floor guard (THROWAWAY) ---------------------------------------
# A raised floor/ridge sits between gate 3 and gate 4 at altitude ~-24.84 m - ABOVE gates 4/5
# (-25.36 / -25.97), so the racing line dips THROUGH it and the drone scrapes (env collisions
# clustered at x=-117..-123). The gate-referenced floor guard (lowest gate - margin = -26.27)
# never fires there. We hold a local altitude floor over the ridge's x-window, then release it so
# the drone can still descend to the low gates 4/5. This hardcodes deterministic course geometry -
# same category as the fixed gate positions - and is Round-1 only. Round 2 needs real terrain
# sensing (no rangefinder/baro in the MAVLink stream; the camera is the only floor-aware sensor).
# Rip out when the course changes.
P1_RIDGE_X_MIN = -131.0     # NED-x window of the raised floor (more negative = further down-track)
P1_RIDGE_X_MAX = -113.0
P1_RIDGE_FLOOR_ALT = -24.3  # min altitude (m, up) to hold inside the window (~0.5 m over the ridge)

# Cross-track
P1_KP_POS = 3.0
P1_KD_VEL = 3.5
P1_LA_TIME = 0.35
P1_LA_MIN = 1.0
P1_LA_MAX = 6.0
P1_LA_WEIGHT = 0.6
