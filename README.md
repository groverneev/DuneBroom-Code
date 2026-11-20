# DuneBroom v2

Software for DuneBroom, a low-cost tracked rover that surveys a beach plot
with a downward-facing camera and reports trash density (items/m²).
Project site: [dunebroom.com](https://dunebroom.com).

## Layout

| Path | What it is |
|---|---|
| `firmware/dunebroom_motor/` | Arduino Uno sketch: drives the tracks through the I2C encoder motor driver; serial command protocol documented at the top of the file |
| `rover/` | Python package that runs on the Raspberry Pi 5 (`Rover` driver class) |
| `config/rover.toml` | Serial port, track wiring and calibration numbers |
| `tools/` | Scripts run on the Pi: `flash.sh` uploads the firmware, `rover_cli.py` tests motion |

## On the Pi

```bash
tools/flash.sh                                 # upload firmware (tracks off the ground)
python3 tools/rover_cli.py ping                # firmware version + battery
python3 tools/rover_cli.py channels            # which driver channel is which track
python3 tools/rover_cli.py drive --ticks 3000  # drive by encoder ticks
```

Needs `arduino-cli` with the `arduino:avr` core, and `python3-serial`.
