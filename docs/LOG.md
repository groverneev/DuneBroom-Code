# DuneBroom logbook

One short entry per day, newest at the top: approximate hours, then a few
high-level bullets on what was done, found and decided. Details live in
commits and `AGENTS.md`.

## 2026-11-24 — ~1.5 h

- Calibrated driving on hardwood: distance (5.36 counts/mm) and turning
  (about 7.4 counts/degree plus a fixed loss per turn, since small turns came
  up short). Needs redoing on sand.
- Right motor briefly lost power; came back after re-seating its plug.

## 2026-11-22 — ~8 h

- Installed the PiPower 5: Pi now runs and charges reliably under load.
  Cooling still inadequate (84 °C under stress).
- Replaced the camera with a better one; fixed the robot's battery issues.
- Switched laptop-to-Pi access to SSH.
- Found the encoder fault: one of the two signals is dead on each motor.
  Workaround: wired each motor's working signal to the Arduino, which now
  counts it (firmware v4). First encoder-measured drive hit its target.
- Next: floor calibration (distance and turn), Pi cooling.

## 2026-11-21 — ~6 h

- Refocused the project on v2 (beach trash-density survey rover); deleted v1.
- Set the plan: roadmap, survey decisions, `AGENTS.md` as shared context.
- Wrote new motor firmware and Pi control software; linked the Pi for remote work.
- Mapped the motors; found the encoders read zero, so switched to timed driving.
- Pi kept browning out: ordered the SunFounder PiPower 5 battery HAT.
