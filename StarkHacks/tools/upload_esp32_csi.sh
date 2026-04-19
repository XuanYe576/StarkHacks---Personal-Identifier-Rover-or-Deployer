#!/usr/bin/env bash
set -euo pipefail

# Build + upload helper for StarkHacks ESP32 CSI firmware.
#
# Usage:
#   ./tools/upload_esp32_csi.sh -p /dev/cu.usbserial-120
#   ./tools/upload_esp32_csi.sh -p /dev/cu.usbserial-120 -m
#   ./tools/upload_esp32_csi.sh -p /dev/cu.usbserial-120 -e adafruit_qtpy_esp32s3_n4r2
#
# Notes:
# - Run from StarkHacks/ directory.
# - Uses `python -m platformio` so PATH does not need `pio`.

ENV_NAME="esp32-s3-devkitc-1"
PORT=""
MONITOR=0
BAUD="921600"

while getopts ":e:p:b:mh" opt; do
  case "$opt" in
    e) ENV_NAME="$OPTARG" ;;
    p) PORT="$OPTARG" ;;
    b) BAUD="$OPTARG" ;;
    m) MONITOR=1 ;;
    h)
      sed -n '1,22p' "$0"
      exit 0
      ;;
    \?)
      echo "Unknown option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

if [[ -z "$PORT" ]]; then
  echo "Error: upload port required. Example: -p /dev/cu.usbserial-120" >&2
  exit 1
fi

if [[ ! -f "platformio.ini" ]]; then
  echo "Error: run this script from StarkHacks/ (platformio.ini not found)." >&2
  exit 1
fi

VALID_ENVS="$(sed -n 's/^\[env:\(.*\)\]$/\1/p' platformio.ini | tr '\n' ' ')"
if [[ -z "$VALID_ENVS" ]]; then
  echo "Error: no [env:*] found in platformio.ini" >&2
  exit 1
fi

if ! echo " $VALID_ENVS " | grep -q " $ENV_NAME "; then
  FIRST_ENV="$(sed -n 's/^\[env:\(.*\)\]$/\1/p' platformio.ini | head -n 1)"
  echo "Warn: env '$ENV_NAME' not found. Switching to '$FIRST_ENV'." >&2
  echo "Valid envs: $VALID_ENVS" >&2
  ENV_NAME="$FIRST_ENV"
fi

echo "[1/2] Building + uploading env=$ENV_NAME port=$PORT"
python -m platformio run -e "$ENV_NAME" -t upload --upload-port "$PORT"

if [[ "$MONITOR" -eq 1 ]]; then
  echo "[2/2] Opening monitor baud=$BAUD"
  python -m platformio device monitor -p "$PORT" -b "$BAUD"
fi
