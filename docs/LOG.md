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
