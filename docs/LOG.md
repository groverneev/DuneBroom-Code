# DuneBroom logbook

Append-only, newest entry at the bottom. One dated entry per work session:
what was done, what was measured, what failed, what was decided. Never edit
old entries; correct them in a new one. Current state lives in `AGENTS.md`.

## 2026-11-22 — PiPower 5 UPS HAT installed

**Done**
- Mounted the SunFounder PiPower 5 HAT on the Pi 5 with its 7.4 V 2000 mAh pack.
- Enabled I2C (`raspi-config nonint do_i2c 0`; it was off). HAT answers at 0x5c.
- Installed the `pipower5` tool 1.2.3 (`sudo python3 install.py`, run with
  `--skip-reboot`; no reboot was needed). HAT firmware 2.0.9. The installer
  also added InfluxDB and a web dashboard (port 34001).
- Started `pipower5.service` (enabled at boot): safe shutdown at 10 % battery.

**Measured**
- On battery at idle: Pi output ~6.7 W, pack current about −1 A at 7.5 V →
  roughly 2 h runtime from full.
- First load test found the HAT reporting input 0 V (charger not seated).
  Re-plugged: input 15.1 V (PD at 15 V).
- 4-core `stress-ng` on the charger, at the 5 W charge setting:

  | t | throttled | temp °C | batt % | batt mA | in W | out W |
  |---|---|---|---|---|---|---|
  | 0 (idle) | 0x0 | 52.1 | 76 | +604 | 11.2 | 4.5 |
  | 1 min | 0x0 | 79.6 | 78 | +608 | 16.2 | 9.2 |
  | 2 min | 0x80008 | 84.0 | 79 | +616 | 15.6 | 9.9 |

  Stopped at 2 min (temp > 82 °C). No undervoltage bits at any point.
- Charge current was capped at ~600 mA = CHARGE SEL DIP at 5 W (both off).
  Set both switches ON (20 W) with a 96 W PD charger: battery current
  ~2.35 A, HAT input ~26 W, tool's max charge current now 2400 mA.

**Results**
- Power: PASS. No undervoltage under load, source stays external, battery
  charges while the Pi works hard. Anker power bank not tested yet.
- Cooling: FAIL. 84 °C after 2 min of stress despite a fan; the HAT on top
  likely blocks airflow.

**Notes**
- The HAT beeps briefly when the charger is unplugged (normal warning).
- Charge only through the HAT's USB-C INPUT port, never the Pi's own USB-C.
- Agents on the Pi need the user's sudo password (asked via a private prompt).

**Next**
- Fix cooling (side photo of the fan/Pi/HAT stack), then re-run the 10-min
  stress test.
- Test with the Anker power bank on INPUT.
- Battery monitoring + low-battery survey guards; chassis mount for Pi + HAT.

## 2026-11-21 — v2 restart: cleanup, firmware, motor diagnostics, power plan

Backfilled 2026-11-22; covers the Fri 11-20 and Sat 11-21 sessions combined.
Time: ~6 h (estimate).

**Done**
- Scoped the project to v2 (survey rover, measures density, no collection).
  Deleted all v1 code and notebooks.
- Wrote the roadmap and decisions now in `AGENTS.md`: start at a marked
  corner, serpentine lanes, drive first / analyze after, all on the Pi offline,
  detector is "an ML model" (architecture not fixed).
- New Uno firmware `firmware/dunebroom_motor` (serial protocol, 600 ms
  watchdog, Pi heartbeat, stall stop, non-blocking moves) and Pi package
  `rover/` + `config/rover.toml` + `tools/rover_cli.py`, `tools/flash.sh`,
  `tools/diag_driver.py`, `tools/diag_interactive.py`.
- Linked the Pi as a T3 environment so laptop agents can run commands on it.
- Set up `AGENTS.md` as the single cross-session context file.

**Measured**
- Tracks: right = M3 (ch 2), left = M4 (ch 3), positive = forward.
- Encoders: zero counts on every channel, with motors running and when
  turned by hand. Encoder boards lit, cables seated.
- Driver in encoder motor type (3): ignores PWM; speed mode on any one
  channel spins both motors at full power. In no-encoder type (0): per-channel
  PWM works; ~30 % doesn't move the tracks, 60 % is slow.
- Motor pack (Hiwonder 11.1 V 6000 mAh, 3S Li-ion): ~12.5 V. Charge socket
  is DC 5.5 × 2.5 mm (Hiwonder spec); needs a 12.6 V CC/CV charger, ~2 A.

**Failed**
- Closed-loop (encoder) driving: unusable until encoders count.
- Pi died when its ad-hoc cell holder ran flat; undervoltage warnings before.

**Decided**
- Firmware defaults to open-loop PWM (`closed_loop = false`); never enable
  closed loop while encoders are broken.
- Separate Pi battery: ordered SunFounder PiPower 5; Anker Nano power bank on
  its input for long days. Motors keep their own pack.

**Next**
- Encoder test: one encoder wired straight to the Uno (VCC→5V, GND, A→D2,
  B→D3), count pulses in firmware.
- Install and test the PiPower 5.

## 2026-11-22 — Encoder bench test: one channel per motor is stuck low

**Done**
- Switched laptop→Pi access to SSH (`ssh dunebroom`, key auth); T3 link is fallback.
- Added `firmware/encoder_test` (interrupt quadrature counter on D2/D3,
  prints edges per channel) and `tools/encoder_watch.py`. `flash.sh` takes
  `SKETCH=...` and finds arduino-cli over non-login SSH.
- Wired each encoder straight to the Uno (driver bypassed, motor pack off),
  turned the track by hand. Reflashed `dunebroom_motor` afterwards.

**Measured**
- Left motor (M4 cable): B pulses (~18.5k edges in 90 s); A held at 0 with
  the Uno pull-up on. Swapping the D2/D3 jumpers moved the fault with the
  wire (Uno pins fine). New jumper: same.
- Right motor (M3 cable): A pulses (~3.2k edges in 30 s); B held at 0.
- M3's port labels run mirrored to M4's (M+ at the bottom). Counting plug
  holes from the top on M3 reverse-powered the right encoder briefly; it
  survived (LED on, A pulses).
- Encoder LED is on the motor's encoder board; labels there are hidden.
- One stuck channel explains the driver's zero counts: quadrature decode
  turns each edge of the live channel into +1 then -1.

**Decided**
- Workaround: the Uno counts the live channel of each motor itself
  (single-channel, direction from the commanded sign). Distance/turn
  resolution ~990 edges per output-shaft turn is ample; sand slip and image
  ground-shift matter more. Replacement 2-channel encoder motors optional.

**Next**
- Tap the live signal wire of each motor cable to the Uno (left → D2,
  right → D3) while the cable stays in the driver; firmware counts it.
