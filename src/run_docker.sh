#!/bin/bash
# Grant container permissions to host X11 display
xhost +local:docker

# Optimize network socket buffers on host for Basler streaming (prevents dropped packets)
sudo sysctl -w net.core.rmem_max=33554432 > /dev/null 2>&1
sudo sysctl -w net.core.rmem_default=33554432 > /dev/null 2>&1

# Build (if modified) and run the interactive container shell
docker compose run --rm multi-cam-env