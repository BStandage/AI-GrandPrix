"""The gate-seeker: flies the published course on vision, heading and
barometric altitude only. No position estimate anywhere.

    seeker.brain      the state machine (pure, unit-tested)
    solvers.seeker    sim adapter (SensorUpdate -> brain -> RCCommand)
    hardware.runtime  Orin adapter (camera + FC bridge -> brain -> MSP)
"""
