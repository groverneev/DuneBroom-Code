#!/usr/bin/env bash
# Compile and upload the motor firmware to the Arduino Uno (run on the Pi).
# Uploading resets the Arduino; keep the tracks off the ground.
set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${1:-$(ls /dev/serial/by-id/*Arduino* 2>/dev/null | head -n1)}"
PORT="${PORT:-/dev/ttyACM0}"

arduino-cli compile --fqbn arduino:avr:uno firmware/dunebroom_motor
arduino-cli upload -p "$PORT" --fqbn arduino:avr:uno firmware/dunebroom_motor
echo "Uploaded to $PORT"
