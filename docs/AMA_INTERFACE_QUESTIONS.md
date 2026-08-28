# AMA questions — drone control interface (draft email)

Subject: Physical Qualifier — flight controller interface questions

Hi all,

Ahead of the AMA, we'd appreciate clarity on how our autonomy stack
interfaces with the flight controller on the physical qualifier drones.
The spec (VADR-TS-004) tells us the airframe runs Betaflight with a UART
link to the Orin NX, but the interface details determine most of our
software architecture, so:

1. **Control protocol** — How do we command the FC over the UART? MSP
   (e.g. MSP_SET_RAW_RC / RC override), CRSF serial-RX emulation, or
   something else? What update rate is expected, and what is the
   staleness/failsafe behavior if our command stream hiccups?

2. **FC configuration** — Is the Betaflight configuration fixed, or can
   teams adjust flight mode (angle/acro), rates/expo, PIDs, and filters?
   If fixed, will the full config diff be published so we can replicate
   it in simulation?

3. **Telemetry** — What comes back over the UART (attitude, gyro,
   battery, etc.), via which messages, and at what rates?

4. **Race/gate feedback** — Is any live race state provided to the drone
   or team during a run (gate-pass ticks, lap times), or is scoring
   entirely external? (Asked before; still unclear in the spec.)

5. **Arming & safety** — What is the arming procedure during a scored
   run? Any geofence, kill-switch, or auto-disarm behavior our stack
   must account for?

6. **Camera** — For the Arducam: do we get exposure/gain control, and is
   there a way to timestamp frames against the IMU clock for sensor
   fusion?

7. **Practice windows** — Are system-identification maneuvers (step
   responses, hover trims) permitted during practice time, and is bench
   /tethered powered testing allowed outside the flight windows?

8. **Firmware** — Which Betaflight version/build will the drones run,
   and will it be pinned by race day?

Thanks — happy to take pointers to existing docs if any of this is
already published.

Brian Standage
