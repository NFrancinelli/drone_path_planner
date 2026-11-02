#!/usr/bin/env bash
# Host side: run_take.sh <world> <cost> <name> [shot]
# Retries up to 3 times when PX4 doesn't take off (take.sh exits 2: no hover).
REPO=$(readlink -f "$(dirname "$(readlink -f "$0")")/../..")
R=$(dirname "$(readlink -f "$0")")  # mounted as /rec; takes land in $R/out
for try in 1 2 3; do
  docker rm -f drone-rec >/dev/null 2>&1
  docker run --rm --name drone-rec --network host --ipc host --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all --device /dev/dri \
    -e SHOT_AFTER="${SHOT_AFTER:-60}" -e RTF="${RTF:-0.2}" -e LEG2="${LEG2:-3.8 7.5}" \
    -v "$REPO/ros2_ws:/ws" -v "$R:/rec" drone-planner-rec:local bash /rec/take.sh "$@"
  rc=$?
  [ $rc -ne 2 ] && exit $rc
  echo "No takeoff (try $try), restarting"
done
exit 2
