#!/usr/bin/env bash
# Build the simulation image. First build takes a while (PX4 + px4_msgs compile).
set -euo pipefail
cd "$(dirname "$0")"
docker build -t drone-planner:latest "$@" .
