#!/usr/bin/env bash
# Disable the LSM6DSOX IIO triggered buffer after a reader has stopped.
set -euo pipefail

DEVICE_NAME="${DEVICE_NAME:-lsm6dsox}"
IIO_ROOT=/sys/bus/iio/devices

iio_device=""
for name_file in "$IIO_ROOT"/iio:device*/name; do
  [[ -e "$name_file" ]] || continue
  if [[ "$(<"$name_file")" == "$DEVICE_NAME" ]]; then
    iio_device="${name_file%/name}"
    break
  fi
done

if [[ -z "$iio_device" ]]; then
  echo "IIO device named '$DEVICE_NAME' was not found." >&2
  exit 1
fi

printf '0\n' | sudo tee "$iio_device/buffer/enable" >/dev/null

echo "IIO buffer disabled"
echo "  device: $iio_device"
echo "  state:  $(<"$iio_device/buffer/enable")"
