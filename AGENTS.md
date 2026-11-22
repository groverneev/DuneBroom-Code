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

## Hardware (measured)

- Raspberry Pi 5 (8 GB), hostname `neev-rpi5-green`, user `groverneev`, repo at
  `~/DuneBroom_Code`. Debian 13, Python 3.13, pyserial, arduino-cli + avr core.
  No OpenCV/ultralytics yet (we can add any package needed; act like you own the pi
  and feel free to do what you need on it)
- Elegoo Uno R3 on USB (`/dev/serial/by-id/*Arduino*`), I2C to a Hiwonder
  4-ch encoder motor driver V1.4 at 0x34. Battery ~12.5 V.
- Two JGB37-3865-520 motors (45:1, 12 V, 150 RPM) with encoder boards.
  **Right track = M3 (ch 2), left track = M4 (ch 3)**, positive = forward.
- **Encoders return zero counts** (motors running or turned by hand) though the
  encoder boards are lit and cables are seated. In encoder motor type the
  driver ignores PWM, and speed mode on any channel spins both motors flat out.
  Firmware therefore runs open-loop PWM: ~30% doesn't move the tracks, 60% is slow.
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
- USB webcam "Full HD webcam" on `/dev/video0`; MJPG up to 1920x1080@30.
  Auto exposure/focus/white balance are on by default (must be locked for surveys).

## Working on the robot

- Code is written on the laptop, pushed to `main`, then `git pull` on the Pi.
- `tools/flash.sh` uploads `firmware/dunebroom_motor` (serial protocol is
  documented at the top of the sketch). `tools/rover_cli.py` and
  `tools/diag_*.py` test motion.
- Laptop agents reach the Pi via T3 linked environment
  `7f7b1199-3089-4593-bf7a-705316869cf1` (delegate_task to its "No project"
  scratch project). The user watches motor tests in person and reports what moved.
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

1. Motion: accurate drive/turn by distance (blocked on encoders).
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

## Status (2026-11-22)

- **Now:** diagnosing encoders. Next test: wire one encoder (VCC->5V, GND,
  A->D2, B->D3) straight to the Uno and count pulses in firmware. If it
  counts, the driver board's encoder inputs are at fault and the Uno counts
  encoders itself; if not, the encoders are dead.
- **Works:** timed open-loop driving (`rover_cli.py run`), watchdog, Pi<->Uno link.
- **Blocked:** distance/angle moves and calibration (need encoders).
- **PiPower 5 installed (2026-11-22):** power side passes. Under 4-core
  stress on the 96 W charger: no undervoltage, source stays external, battery
  charges. Not yet tested with the Anker. **Cooling fails:** the SoC hit 84 °C
  after 2 min of stress (soft throttle) despite a fan; the HAT on top likely
  blocks airflow. Fix the cooling, then re-run the 10-min stress test. Still to do:
  battery monitoring (Pi % + motor pack voltage) and survey guards (refuse to
  start / stop and save when low), and a chassis mount for the Pi + HAT.
