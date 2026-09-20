# New drone setup

Everything to take a drone from the box to flight-ready. ~30 minutes.
Replace `dNN` with the drone's number throughout.

Props OFF for all of it.

---

## 1. Get in

Plug the **Jetson's** micro-USB (next to the barrel jack, not the FC's) into
the laptop.

Try SSH first:

```
ssh dcl@192.168.55.1          # password dcl
```

If it drops repeatedly, use the serial console instead - it does not drop.
PuTTY -> Serial -> the new COM port -> 115200 -> Open -> log in `dcl` / `dcl`.
Find the port with:

```
Get-PnpDevice -PresentOnly -Class Ports
```

Both Jetsons answer on 192.168.55.1 with different host keys, so swapping
drones trips a host-key warning. Clear it:

```
ssh-keygen -R 192.168.55.1
```

## 2. WiFi

```
sudo nmcli device wifi connect 'AI Grand Prix' password 'Anduril2421'
ip -br addr show wlP1p1s0
```

Write the address down. It is DHCP and can change.

## 3. Clock

They boot thinking it is 2023, which corrupts every log filename.

```
sudo timedatectl set-ntp true
timedatectl
```

## 4. Identify the board

All of them report hostname `dcl-orin`, so the prompt is the only label.

```
cat >> ~/.bashrc <<'EOF'

# ---- AIGP ----
export DRONE=dNN
PS1="\[\e[1;33m\][$DRONE]\[\e[0m\] \u@\h:\w\$ "
MSP="python3 $HOME/target/msp/msp_bench.py --port /dev/ttyTHS1"
alias fc-info='$MSP info'
alias fc-telem='$MSP telemetry --hz 20'
alias fc-rc='$MSP rc'
alias cam-check='v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=RG10 --stream-mmap --stream-count=100 --stream-to=/dev/null'
alias cam-view='~/target/live-view.py'
alias wifi-ip='ip -br addr show wlP1p1s0'
alias whichdrone='echo $DRONE; $MSP info | head -3'
EOF

exec bash
```

Worked: the prompt now starts with `[dNN]`.

**Also put the number on the frame with a paint pen.** Two unlabelled drones
cost us hours on 2026-09-19.

## 5. Free the UART

Once per board. The Linux console owns it by default.

```
~/target/msp/setup_jetson_uart.sh                 # inspect only
sudo ~/target/msp/setup_jetson_uart.sh --apply
```

## 6. Flight controller link

```
fc-info
```

Worked: firmware, sensors, battery voltage, and why it will not arm.

`No MSP reply` with the Jetson side checking clean: close Betaflight if it is
connected, then **power cycle the drone**. That fixed d44.

## 7. Camera

```
ls /dev/video0
cam-check
```

Worked: 100 frames captured at 30 fps.

## 8. Our code

From the laptop, not the drone. 2.7 MB, not the whole repo:

```
cd ~/GitRepos/AI-GrandPrix
tar czf - --exclude='__pycache__' --exclude='*.pyc' \
  src/hardware src/perception src/raceline src/seeker src/solvers src/common \
  config data \
| ssh dcl@<drone ip> "mkdir -p ~/AI-GrandPrix && tar xzf - -C ~/AI-GrandPrix"
```

Check it:

```
cd ~/AI-GrandPrix/src
python3 -c "import numpy, cv2, serial; print(cv2.__version__)"
```

Do NOT `pip install opencv-python` - JetPack's build has GStreamer, the wheel
does not.

---

## 9. Betaflight config

FC's own USB into the laptop. Betaflight Configurator -> pick the **COM port
with vendor ID 0483** (STM32), not the Jetson's 0955 -> Connect.

Sanity check you are really connected: the traffic counters bottom-left must
be non-zero and the model must move when you tilt the drone. "Connect
(Virtual)" looks identical to success and is not.

CLI tab:

```
version
diff all                 <- then click Save to File. Do this BEFORE changing anything.
map                      <- must say AETR1234
aux                      <- read it, see below
get msp_override_channels_mask
```

### The override mask

```
set msp_override_channels_mask = 15
save
```

Default is 11, which leaves throttle on the radio AND lets MSP write the
override switch itself, so the pilot cannot take control back.

After the reboot, verify:

```
get msp_override_channels_mask       <- must read 15
```

### ANGLE mode

Neither d44 nor d45 had ANGLE on a switch. The follower sends angle sticks, so
it must be on. We hard-code it always-on:

```
aux <first free row> 1 5 900 2100 0 0
save
```

**The row number differs per drone.** Read the `aux` output first and use the
first row showing `0 0 900 900`. On d44 that was row 2, on d45 row 5. Writing
to an occupied row silently deletes a live flight mode.

`1` = ANGLE, `5` = AUX6, `900 2100` = always active.

Verify:

```
aux
```

Row you wrote reads `1 5 900 2100`, every other row unchanged.

---

## 10. Calibrate the camera

**Per aircraft. None of these numbers transfer between drones.** Four of them,
and every one goes on every flight command.

### 10a. Intrinsics, from the printed checkerboard

Print `out/caltarget/checkerboard_letter_25mm.png` at 100 %, tape it FLAT to a
wall, and measure a square across five squares divided by five - printers
scale. Ours came out 25.7 mm against a nominal 25.

```
cd ~/AI-GrandPrix/src
python3 -m hardware.camcal_board grab --want 20
```

Move the DRONE between captures: closer and further, angled, and get the board
into the frame CORNERS - that is where distortion lives.

```
python3 -m hardware.camcal_board solve --square-mm 25.7
```

Worked: reprojection RMS under 0.5 px. Write down `fy`, `hfov`, `cx`, `cy`.

The principal point matters as much as the focal length: it is the optical
axis, and it is NOT the image centre. On d45 it was 27 px off in both axes,
which is a constant -1.8 deg horizontal bias on every single camera fix,
always the same way - about 0.26 m of lateral error at 8 m.

### 10b. Mount tilt, from the same board

The checkerboard cannot give this: it knows the lens, not how the camera is
bolted to the airframe.

1. Level the drone ON THE FLOOR. Body pitch goes straight into the answer:

```
python3 ~/target/msp/msp_bench.py --port /dev/ttyTHS1 telemetry --hz 20
```

Shim until roll and pitch are within +-1.

2. Measure, all from the same floor, in metres:
   - **dist**  lens to wall, perpendicular
   - **lens-h** lens height
   - **target-h** height of the checkerboard's CENTRE

3. Run it at ~40 inches and again at ~70 inches:

```
python3 -m hardware.camtilt --dist 1.016 --lens-h 0.830 --target-h 1.299     --fy 830 --cy 387
```

The two answers must agree within 2 deg. d45 gave 20.6 at 40 in and 19.4 at
70 in -> **20 deg**.

Do NOT mark the frame edges with tape instead. We tried: the edges land on a
vertical wall through tan(theta +- vhalf), which is not symmetric, and any
crop in the browser biases one edge. It gave anything from 21 to 30 deg on the
same setup.

You do not need to hit a target angle. The angle only has to be KNOWN - the
plan gets built for whatever it measures. Only re-adjust the hinge if it is
above ~30 deg, where gates at your own altitude drop out of frame in level
flight.

### 10c. Heading drift

No magnetometer, so the heading is gyro-integrated and walks.

```
python3 -m hardware.bench --port /dev/ttyTHS1 drift --seconds 60
```

Dead still, on the FLOOR (a table reads worse - d45 gave 4.0 deg/min on a
table, 3.0 on the floor), after the board has been powered 10+ minutes so the
gyro is thermally settled.

Whatever it reads goes into `--heading-drift-dpm`, which subtracts it linearly
over the run. The drift is only roughly linear, so it halves the error rather
than removing it.

### 10d. Write them down

```
DRONE dNN
  --fy               ____
  --cam-hfov         ____
  --cam-tilt         ____
  AIGP_CAM_CX        ____
  AIGP_CAM_CY        ____
  --heading-drift-dpm ____
```

Every flight command for this drone:

```
AIGP_CAM_CX=<cx> AIGP_CAM_CY=<cy> python3 -m hardware.runtime --port /dev/ttyTHS1 --map-north here   --cam-tilt <tilt> --fy <fy> --cam-hfov <hfov>   --heading-drift-dpm <drift>   --pilot follower --config <rung toml> --traj <rung plan> --dry-run
```

Without the two env vars you carry the boresight bias into every fix.

---

## 11. The override test

Props off, battery in, transmitter on.

```
cd ~/AI-GrandPrix/src
python3 -m hardware.override_test --port /dev/ttyTHS1
```

1. MSP OVERRIDE on -> `MSP has the sticks`
2. MSP OVERRIDE off -> `PILOT has the sticks`  **this is the test**
3. ARM on, then off -> `armed` goes YES then no

Prints PASS or INCOMPLETE. INCOMPLETE means do not fly this drone.

---

## 12. Record it

Add to the fleet table: drone number, serial, WiFi address, mask verified,
ANGLE row, override test result.

## Quick checklist

```
[ ] logged in (serial or USB)
[ ] WiFi joined, address written down
[ ] clock correct
[ ] .bashrc prompt shows [dNN]
[ ] number painted on the frame
[ ] UART freed
[ ] fc-info works
[ ] cam-check captures 100 frames
[ ] repo copied, cv2 imports
[ ] diff all saved
[ ] map = AETR1234
[ ] mask = 15
[ ] ANGLE row added and verified
[ ] camcal_board: fy, hfov, cx, cy   (RMS under 0.5 px)
[ ] camtilt at two distances, agreeing within 2 deg
[ ] heading drift measured on the floor, warmed up
[ ] all six numbers written down
[ ] override test PASS
```
