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
  I2C was enabled with raspi-config. `dtparam=uart0=on` (2026-11-26) gives
  `/dev/ttyAMA0` on GPIO14/15 for the BNO055; serial0/console stay on the
  debug port (ttyAMA10). Long days: Anker Nano power bank on the
  INPUT port. Motors stay on the Hiwonder 11.1 V 6000 mAh pack (separate charger).
- Camera: Logitech C920 PRO (USB, `/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_*-index0`),
  pointing down. MJPG up to 1920x1080@30. Has manual exposure (100 us units),
  gain, focus 0-250 (~45 at current height), white balance 2000-6500 K.
  `rover/camera.py` locks them (auto settles, then frozen; exposure capped by
  `[camera] exposure_max`). OpenCV 4.10 (python3-opencv) installed on the Pi.

## Working on the robot

- Code is written on the laptop, pushed to `main`, then `git pull` on the Pi.
- `tools/capture.py` saves locked frames + odometry to `data/captures/`
  (gitignored); `tools/camera_cal.py` calibrates the camera with
  `docs/checkerboard_25mm.pdf`; `tools/live_view.py` streams the camera to
  http://neev-rpi5-green.local:8000 with a live tilt readout for aiming (`--snap S` auto-saves lens views); `tools/imu.py` checks the BNO055 and runs
  gyro turns; `tools/power.py` shows both batteries; `tools/diag_i2c_wiggle.py`
  hunts loose Uno-driver wires.
- `tools/survey.py plan|run --width W --length L` (metres) plans/runs a
  survey; `run --dry-run` uses the simulated rover (`rover/sim.py`).
- Tests: `python3 tests/test_gyro.py` and `tests/test_survey.py` on the Pi (no pytest; laptop lacks pyserial).
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
6. Debris detector: try zero-shot open-vocabulary models (SAM 3, Grounding
   DINO, OWLv2...) first, measured on a small hand-labelled beach set. Train
   a small model (pre-labelled by the zero-shot one) only if that falls short.
   May run on the laptop if too slow for the Pi.
7. Count each item once (new-ground strips + lane bands).
8. Results: density, per-item CSV, detection map, summary JSON.
9. Evaluation tools: ground-truth format, replay, precision/recall, MAPE, CV.
10. Some form of Field UI

## Status (2026-11-27)

- **Now:** motion calibrated on hardwood: ticks_per_mm = 5.36 (933.5 mm,
  straight, repeatable). Turns: 7.54 ticks/deg, but skid turns scatter
  ~+-10% run to run (4 x 90 deg came out 450 deg). Encoder-only turning
  isn't accurate enough for lane ends. Fix: Adafruit BNO055 IMU ordered
  2026-11-25 (arrives ~11-29); plan to wire it to the Pi in UART mode (PS1
  high) to avoid Pi I2C clock-stretching trouble. Code ready, untested on
  hardware: `rover/imu.py` (UART, IMUPLUS mode, unwrapped clockwise yaw) and
  `rover/heading.py` (encoder turn, then gyro-measured corrections; learns
  the measured/commanded ratio; turns to absolute headings; lanes driven in
  segments re-squared between them). Simulated: 98% of turns within 1.5 deg.
  Pi UART enabled. Sunday: wire it, `tools/imu.py watch`, then `turn`.
  Recheck distance on sand.
- **Camera calibration (roadmap 3) in progress:** aimed 2026-11-27 with
  `tools/live_view.py` + the printed board: camera ~262 mm above the floor
  (not the ~450 mm first assumed), level both ways (was ~5 deg), one frame
  sees ~369 x 208 mm at 0.19 mm/px (rough: lens not yet fitted). Mast base
  fitting, camera clip and other loose parts hot glued 2026-11-27. `[camera] focus` fixed at 45.
  19 flat views taken (lens1); a fit from flat views alone is wrong (gave a 40 deg
  lens at 52 cm), so `solve` now needs 5+ views tilted >= 12 deg. Next: add tilted
  views (`live_view.py --snap lens1 --min-tilt 12`), `solve`, `ground`.
- **Watch: flaky I2C** between Uno and driver. 2026-11-25: `ERR I2C` at
  startup, then a lost stop left the robot driving forward after DONE TARGET
  until a new connection reset the Uno. Firmware v5 retries stops and
  re-sends stop every 100 ms while idle; v6 also retries speed writes.
  Wiggle test (`tools/diag_i2c_wiggle.py`) traced it to the SDA wire
  (SCL/GND clean); after rewiring SDA it passed. Glue/secure the I2C wires.
- **Works:** `drive --mm` / `turn --deg`, encoder-counted moves (firmware v4: Uno counts the live channel
  of each motor on D2/D3; `drive --ticks` reached target 1997/2017 on lifted
  tracks), timed open-loop driving, watchdog, Pi<->Uno link.
- **Capture (roadmap 2) started 2026-11-25:** camera locking and frame
  capture with odometry work (5 fps, 1080p). Indoors the 8 ms exposure cap
  forces max gain (noisy); check frames in daylight. Not yet run while driving.
- **Survey runner (roadmap 4) written, simulated only:** `rover/survey.py`
  plans serpentine lanes from the camera footprint (20% side overlap),
  drives them in 500 mm segments steering back onto each lane's line by
  odometry (encoders + gyro yaw), checks the power guard between moves, and
  saves frames.csv with pose/heading/phase/lane per frame, events.jsonl and
  summary.json under `data/surveys/`. Dry runs end within ~3 cm of the
  planned line. To measure: `[survey] camera_ahead_mm`. Stopping every
  segment may be slow/jerky: revisit after the blur test.
- **Power guards:** `rover/power.py` reads Pi % and motor pack V; PowerGuard
  is ready for the survey runner: one minimum per battery (Pi 12 %, motor
  10.5 V) for both starting and stopping.
- **Watch: Wi-Fi drops:** the Pi fell off Wi-Fi for a few minutes on
  2026-11-27 without rebooting; probably Wi-Fi power saving. Disable it if it recurs.
- **Blocked:** nothing.
- **PiPower 5 installed (2026-11-22):** power side passes. Under 4-core
  stress on the 96 W charger: no undervoltage, source stays external, battery
  charges. Not yet tested with the Anker. **Cooling passes (2026-11-27):**
  official Active Cooler fitted under the HAT; 10-min 4-core stress on
  battery levels at 73-75 °C, 2.4 GHz throughout, throttled=0x0, fan ~8800
  rpm (was 84 °C and throttling). Still to do: a chassis mount for the Pi + HAT.
