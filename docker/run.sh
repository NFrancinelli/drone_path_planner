#!/usr/bin/env bash
# Start (or attach to) the dev container with the ROS workspace mounted at /ws.
# Usage: docker/run.sh [command...]   (defaults to an interactive shell)
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NAME=drone-planner

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    exec docker exec -it "$NAME" bash -ic "${*:-bash}"
fi

GPU_ARGS=()
if docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia'; then
    GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi
# Mesa GPUs (e.g. the Intel iGPU driving the screen on hybrid laptops): RViz renders here.
if [ -d /dev/dri ]; then
    GPU_ARGS+=(--device /dev/dri)
    for group in video render; do
        gid=$(getent group "$group" | cut -d: -f3) && [ -n "$gid" ] && GPU_ARGS+=(--group-add "$gid")
    done
fi

# Let the container open windows (RViz, Gazebo GUI) on the host display. Share the X auth
# cookie (also what XWayland on Wayland desktops uses); fall back to xhost if there is none.
X_ARGS=()
if [ -n "${XAUTHORITY:-}" ] && [ -f "$XAUTHORITY" ]; then
    X_ARGS=(-v "$XAUTHORITY:/tmp/.Xauthority:ro" -e XAUTHORITY=/tmp/.Xauthority)
elif command -v xhost >/dev/null; then
    xhost +local:docker >/dev/null
fi

exec docker run -it --rm --name "$NAME" \
    --network host --ipc host \
    "${GPU_ARGS[@]}" \
    "${X_ARGS[@]}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$REPO_ROOT/ros2_ws:/ws" \
    drone-planner:latest bash -ic "${*:-bash}"
