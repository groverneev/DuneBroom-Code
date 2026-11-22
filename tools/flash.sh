#!/usr/bin/env bash
# Compile and upload firmware to the Arduino Uno (run on the Pi).
# SKETCH=firmware/encoder_test tools/flash.sh  flashes another sketch.
# Uploading resets the Arduino; keep the tracks off the ground.
set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${1:-$(ls /dev/serial/by-id/*Arduino* 2>/dev/null | head -n1)}"
PORT="${PORT:-/dev/ttyACM0}"
SKETCH="${SKETCH:-firmware/dunebroom_motor}"

arduino-cli compile --fqbn arduino:avr:uno "$SKETCH"
arduino-cli upload -p "$PORT" --fqbn arduino:avr:uno "$SKETCH"
echo "Uploaded to $PORT"
