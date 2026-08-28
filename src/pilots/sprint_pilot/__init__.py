"""Sprint pilot: steady_pilot's stop-and-align machinery plus a bang/counter-bang transit layer.

Fork of steady_pilot v2 (the 2:10 VQ2 qualifier, byte-preserved next door). Between gates - and
only while the lock is confirmed and commit-grade aligned - it leans hard (SPRINT), holds a cruise
ceiling, and counter-leans (BRAKE) sized by a dead-reckoned forward speed and the blob-area range
proxy, arriving at the ALIGN boundary at creep speed where steady's proven machinery takes over.
Fail-open: every abort path degenerates to exactly what steady would have done.
"""

from .sprint_pilot import update_sprint_control

__all__ = ["update_sprint_control"]
