"""On-drone runtime: the flight-controller link and everything that only
exists on the Archer (Betaflight over MSP on the Orin's UART).

    hardware.msp     MSP v1 protocol client, serial or TCP transport
    hardware.bridge  RC-out / telemetry-in thread with the safety rules
    hardware.bench   python -m hardware.bench --port /dev/ttyTHS1 info

The same code talks to the sim's Betaflight SITL over TCP (port 5761),
which is how it is tested before the drone exists.
"""
