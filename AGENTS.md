# DuneBroom v2

Tracked rover that surveys a beach plot with a downward camera and reports
trash density (items/m²). Measures only; does not collect. v1 (trash
collector) is deleted; ignore it. Project site source: `../DuneBroom-Website`.
Information in the website might not be up to date.
The research plan is a rough draft; drop requirements that don't serve the goal.
(You don't have this plan)
Looking at a picture of the robot from the dunebroom website can be helpful, ask me for
other pictures if needed

## Rules

- No AI co-author trailers (`Co-Authored-By:` etc.) or "Generated with ..."
  lines in commits or PR descriptions.
- Motor tests: tracks off the ground, and the user must confirm before any
  command that moves motors.
- Never set `closed_loop = true` while encoders are broken: the driver then
  runs both motors at full power regardless of channel.
- Update **Status** below in the same commit as the work it describes.
- If a tool or workflow default gets in the way of the work (slow, indirect,
  pointless here), say so plainly and propose the better way; don't silently
  work around the user. The user's explicit choices override those defaults.
- Logbook: `docs/LOG.md`, one short entry per day, newest at the top (`## date — ~N h`, then a
  few high-level bullets). Update it with each day's work; no details there.
  Keep this file to current state.

## Hardware (measured)

- Raspberry Pi 5 (8 GB), hostname `neev-rpi5-green`, user `groverneev`, repo at
  `~/DuneBroom_Code`. Debian 13, Python 3.13, pyserial, arduino-cli + avr core.
  No OpenCV/ultralytics yet (we can add any package needed; act like you own the pi
  and feel free to do what you need on it)
- Elegoo Uno R3 on USB (`/dev/serial/by-id/*Arduino*`), I2C to a Hiwonder
  4-ch encoder motor driver V1.4 at 0x34. Battery ~12.5 V.
- Two JGB37-3865-520 motors (45:1, 12 V, 150 RPM) with encoder boards.
  **Right track = M3 (ch 2), left track = M4 (ch 3)**, positive = forward.
- **Encoders: one channel per motor is stuck low** (bench test 2026-11-22):
  left A dead / B live, right B dead / A live; LEDs on. The driver's
  quadrature decode therefore reads 0. In encoder motor type the driver
  ignores PWM, and speed mode on any channel spins both motors flat out.
  Firmware runs open-loop PWM: ~30% doesn't move the tracks, 60% is slow.
  Driver ports M3/M4 have mirrored pin labels: M4 = M+ VCC A B GND M- top to
  bottom, M3 the reverse. Count plug holes from the M+ end.
- Pi power: SunFounder PiPower 5 UPS HAT (I2C 0x5c, fw 2.0.9) on the GPIO
  header, 7.4 V 2000 mAh pack. Charge via the HAT's USB-C **INPUT** port (PD,
  negotiates 15 V), never the Pi's own USB-C. CHARGE SEL DIP both ON = 20 W
  (~2.35 A measured); with 96 W charger the HAT draws ~26 W total. Pi draws
  ~4.5 W idle, ~10 W at 4-core stress; ~1 A from the pack at idle on battery
  (~2 h runtime from full). Buzzer beeps when input is unplugged (normal).
  `pipower5` CLI 1.2.3 (`-a` full status, `-bp` %, `-ichg`, `-ii`);
  `pipower5.service` enabled, safe shutdown at 10 %; dashboard on :34001.
  I2C was enabled with raspi-config. Long days: Anker Nano power bank on the
  INPUT port. Motors stay on the Hiwonder 11.1 V 6000 mAh pack (separate charger).
- Camera: Logitech C920 PRO (USB, `/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_*-index0`),
  pointing down. MJPG up to 1920x1080@30. Has manual exposure (100 us units),
  gain, focus 0-250 (~45 at current height), white balance 2000-6500 K.
  `rover/camera.py` locks them (auto settles, then frozen; exposure capped by
  `[camera] exposure_max`). OpenCV 4.10 (python3-opencv) installed on the Pi.

## Working on the robot

- Code is written on the laptop, pushed to `main`, then `git pull` on the Pi.
- `tools/capture.py` saves locked frames + odometry to `data/captures/`
  (gitignored); `tools/power.py` shows both batteries; `tools/diag_i2c_wiggle.py`
  hunts loose Uno-driver wires.
- `tools/flash.sh` uploads `firmware/dunebroom_motor` (serial protocol is
  documented at the top of the sketch). `tools/rover_cli.py` and
  `tools/diag_*.py` test motion.
- Laptop agents reach the Pi by SSH: `ssh dunebroom` (laptop `~/.ssh/config`,
  key `~/.ssh/dunebroom_pi`, host `neev-rpi5-green.local`). The user chose SSH
  over the T3 linked environment (`7f7b1199-3089-4593-bf7a-705316869cf1`),
  which remains a fallback. The user watches motor tests in person and reports what moved.
- Track wiring and calibration live in `config/rover.toml`; recalibrating
  never needs a reflash (firmware works in encoder ticks).

## Decisions

- Survey: rover starts at a marked plot corner facing lane 1; plot size is
  entered; position comes from odometry (and later image ground-shift).
- Serpentine lanes (back and forth), not same-direction lanes: half the driving.
- Drive first, analyze after: during the survey only capture frames + odometry;
  detection, de-duplication and density run as a batch on the Pi afterwards.
  The same code replays saved surveys on a laptop.
- Everything runs on the Pi with no internet; only model training is off-board.
- Arduino owns motor safety: 600 ms watchdog (Pi heartbeat), stall stop,
  non-blocking moves.

## Rough Roadmap

1. Motion: accurate drive/turn by distance (encoders work via the Uno; needs calibration).
2. Capture: locked camera settings, frames saved with timestamps + odometry.
3. Camera calibration: px/cm, footprint, lens distortion -> config.
4. Dry-run lanes without ML; check coverage/area; these frames become training data.
5. Ground shift via phase correlation; surveyed-area accumulation.
6. Debris detector: an ML model trained on our own images (plus public data
   if it helps), fast enough on the Pi; architecture not fixed.
7. Count each item once (new-ground strips + lane bands).
8. Results: density, per-item CSV, detection map, summary JSON.
9. Evaluation tools: ground-truth format, replay, precision/recall, MAPE, CV.
10. Some form of Field UI

## Status (2026-11-25)

- **Now:** motion calibrated on hardwood: ticks_per_mm = 5.36 (933.5 mm,
  straight, repeatable). Turns: 7.54 ticks/deg, but skid turns scatter
  ~+-10% run to run (4 x 90 deg came out 450 deg). Encoder-only turning
  isn't accurate enough for lane ends. Fix: Adafruit BNO055 IMU ordered
  2026-11-25 (arrives ~11-29); plan to wire it to the Pi in UART mode (PS1
  high) to avoid Pi I2C clock-stretching trouble, then turn by heading.
  Recheck distance on sand.
- **Watch: flaky I2C** between Uno and driver. 2026-11-25: `ERR I2C` at
  startup, then a lost stop left the robot driving forward after DONE TARGET
  until a new connection reset the Uno. Firmware v5 retries stops and
  re-sends stop every 100 ms while idle. Also likely explains the right motor
  (M3) going silent on 2026-11-24. Re-seat/secure the SDA/SCL/GND wires.
- **Works:** `drive --mm` / `turn --deg`, encoder-counted moves (firmware v4: Uno counts the live channel
  of each motor on D2/D3; `drive --ticks` reached target 1997/2017 on lifted
  tracks), timed open-loop driving, watchdog, Pi<->Uno link.
- **Capture (roadmap 2) started 2026-11-25:** camera locking and frame
  capture with odometry work (5 fps, 1080p). Indoors the 8 ms exposure cap
  forces max gain (noisy); check frames in daylight. Not yet run while driving.
- **Power guards:** `rover/power.py` reads Pi % and motor pack V; PowerGuard
  is ready for the survey runner (start >= 30 % / 11.1 V, stop < 15 % / 10.5 V).
- **Blocked:** nothing.
- **PiPower 5 installed (2026-11-22):** power side passes. Under 4-core
  stress on the 96 W charger: no undervoltage, source stays external, battery
  charges. Not yet tested with the Anker. **Cooling fails:** the SoC hit 84 °C
  after 2 min of stress (soft throttle) despite a fan; the HAT on top likely
  blocks airflow. Official Pi 5 Active Cooler ordered 2026-11-25; needs
  ~12 mm under the HAT. Fit it, then re-run the 10-min stress test. Still to
  do: a chassis mount for the Pi + HAT.
