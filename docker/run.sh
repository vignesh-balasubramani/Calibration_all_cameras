#!/usr/bin/env bash
# Build (if needed) and enter the calibration container with camera + display access.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

# Let the container draw on the host X display; revoke again on exit.
xhost +local:docker >/dev/null
trap 'xhost -local:docker >/dev/null' EXIT

# Bigger socket buffers prevent dropped packets on Basler GigE streams.
# Needs root; a failure only degrades streaming, so warn instead of aborting.
if ! sudo -n sysctl -qw net.core.rmem_max=33554432 net.core.rmem_default=33554432 2>/dev/null; then
    echo "[WARN] Could not raise net.core.rmem_* (needs passwordless sudo). Basler GigE may drop packets." >&2
    echo "       Run once manually: sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=33554432" >&2
fi

docker compose run --rm --build calibration
