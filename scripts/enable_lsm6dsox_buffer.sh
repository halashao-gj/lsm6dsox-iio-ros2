#!/usr/bin/env bash
# Enable the LSM6DSOX hardware-FIFO IIO buffer for the ROS 2 publisher.
set -euo pipefail

DEVICE_NAME="${DEVICE_NAME:-lsm6dsox}"
BUFFER_LENGTH="${BUFFER_LENGTH:-128}"
FIFO_WATERMARK="${FIFO_WATERMARK:-4}"
TRIGGER_NAME="${TRIGGER_NAME:-}"
IIO_ROOT=/sys/bus/iio/devices

if ! [[ "$BUFFER_LENGTH" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUFFER_LENGTH must be a positive integer, got: $BUFFER_LENGTH" >&2
  exit 1
fi

if ! [[ "$FIFO_WATERMARK" =~ ^[1-9][0-9]*$ ]] ||
   (( FIFO_WATERMARK > 255 )); then
  echo "FIFO_WATERMARK must be between 1 and 255, got: $FIFO_WATERMARK" >&2
  exit 1
fi

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

if [[ -z "$TRIGGER_NAME" ]]; then
  for name_file in "$IIO_ROOT"/trigger*/name; do
    [[ -e "$name_file" ]] || continue
    candidate="$(<"$name_file")"
    if [[ "$candidate" == "$DEVICE_NAME"* ]]; then
      TRIGGER_NAME="$candidate"
      break
    fi
  done
fi

if [[ -z "$TRIGGER_NAME" ]]; then
  echo "No IIO trigger matching '$DEVICE_NAME*' was found." >&2
  exit 1
fi

device_node="/dev/${iio_device##*/}"
if [[ ! -e "$device_node" ]]; then
  echo "IIO character device '$device_node' was not found." >&2
  exit 1
fi

write_sysfs() {
  local value=$1
  local path=$2
  printf '%s\n' "$value" | sudo tee "$path" >/dev/null
}

# Changing scan layout requires the buffer to be off first.
write_sysfs 0 "$iio_device/buffer/enable"

for scan_enable in "$iio_device"/scan_elements/*_en; do
  [[ -e "$scan_enable" ]] || continue
  write_sysfs 1 "$scan_enable"
done

write_sysfs "$TRIGGER_NAME" "$iio_device/trigger/current_trigger"
write_sysfs "$BUFFER_LENGTH" "$iio_device/buffer/length"
write_sysfs "$FIFO_WATERMARK" "$iio_device/buffer/watermark"
write_sysfs 1 "$iio_device/buffer/enable"

if [[ ! -r "$device_node" ]]; then
  echo "IIO character device '$device_node' is not readable by $(id -un)." >&2
  echo "Install config/udev/99-lsm6dsox-iio.rules and add the user to the iio group." >&2
  exit 1
fi

echo "IIO buffer enabled"
echo "  device:  $iio_device"
echo "  node:    $device_node"
echo "  trigger: $(<"$iio_device/trigger/current_trigger")"
echo "  length:  $(<"$iio_device/buffer/length")"
echo "  watermark: $(<"$iio_device/buffer/watermark")"
echo "  hw watermark: $(<"$iio_device/buffer/hwfifo_watermark")"
echo "  state:   $(<"$iio_device/buffer/enable")"
