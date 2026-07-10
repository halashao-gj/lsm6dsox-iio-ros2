#!/usr/bin/env bash
# Validate the LSM6DSOX module, IIO buffer, ROS publisher, and systemd unit.
set -euo pipefail

SERVICE="${SERVICE:-lsm6dsox-ros.service}"
DEVICE_NAME="${DEVICE_NAME:-lsm6dsox}"
IIO_ROOT=/sys/bus/iio/devices
cleanup_needed=false
topic_output=""
repo_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

if [[ -d "$repo_root/install" ]]; then
  workspace_root="$repo_root"
elif [[ -d "$repo_root/../../install" ]]; then
  workspace_root="$(CDPATH= cd -- "$repo_root/../.." && pwd)"
else
  workspace_root="${ROS_WORKSPACE:-$repo_root}"
fi

cleanup() {
  [[ -z "$topic_output" ]] || rm -f "$topic_output"
  if "$cleanup_needed"; then
    sudo systemctl stop "$SERVICE" || true
  fi
}
trap cleanup EXIT

find_device() {
  local name_file
  for name_file in "$IIO_ROOT"/iio:device*/name; do
    [[ -e "$name_file" ]] || continue
    if [[ "$(<"$name_file")" == "$DEVICE_NAME" ]]; then
      printf '%s\n' "${name_file%/name}"
      return 0
    fi
  done
  return 1
}

device="$(find_device)" || {
  echo "IIO device '$DEVICE_NAME' was not found" >&2
  exit 1
}
node="/dev/${device##*/}"

echo '== module and IIO device =='
modinfo lsm6dsox_driver | sed -n '1,6p'
printf 'device: %s\nnode: %s\n' "$device" "$node"

echo '== standalone buffer frame =='
sudo systemctl stop "$SERVICE"
"$repo_root/scripts/enable_lsm6dsox_buffer.sh"
if ! timeout 3 dd if="$node" of=/dev/null bs=24 count=1 status=none; then
  echo 'IIO buffer did not return a frame within 3 seconds' >&2
  exit 1
fi
"$repo_root/scripts/disable_lsm6dsox_buffer.sh"

echo '== service and ROS frame =='
sudo systemctl start "$SERVICE"
cleanup_needed=true
sleep 2
systemctl --no-pager --full status "$SERVICE"
set +u
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"
set -u
topic_output="$(mktemp)"
# ros2 topic echo is a Python process.  Keep stdout unbuffered because timeout
# may otherwise terminate it after it received a message but before redirected
# output reaches the temporary file.
timeout 8 env PYTHONUNBUFFERED=1 ros2 topic echo --qos-reliability best_effort --once /imu/data >"$topic_output" || true
cat "$topic_output"
if ! grep -q '^header:' "$topic_output"; then
  echo '/imu/data was not published within 8 seconds' >&2
  exit 1
fi

echo 'LSM6DSOX pipeline validation passed'
