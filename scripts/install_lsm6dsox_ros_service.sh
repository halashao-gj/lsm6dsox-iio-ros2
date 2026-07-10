#!/usr/bin/env bash
# Install the manual-start systemd service. It deliberately does not enable it.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="$repo_root/systemd/lsm6dsox-ros.service"
unit_target=/etc/systemd/system/lsm6dsox-ros.service

if [[ ! -f "$unit_source" ]]; then
  echo "Service file not found: $unit_source" >&2
  exit 1
fi

sudo install -m 0644 "$unit_source" "$unit_target"
sudo systemctl daemon-reload

echo "Installed $unit_target"
echo "The service remains disabled. Start it manually with:"
echo "  sudo systemctl start lsm6dsox-ros.service"
