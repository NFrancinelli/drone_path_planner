#!/usr/bin/env bash
# One recorded take, inside the recording container: take.sh <world> <unknown_cost> <name> [shot]
# Leg 2 goes back to LEG2 ("x y", default "3.8 7.5"; "6 -7.5" in factory_detour).
# Writes /rec/out/<name>_top.mp4 (overhead, 1920x1080) and <name>_chase.mp4 (960x540), both at
# wall-clock speed (RTF times real time, 0.2 by default), plus <name>.log and <name>_load.txt.
# shot: fly leg 1 for a while and grab one frame of each view instead.
set +u
WORLD=$1; COST=$2; NAME=$3; MODE=${4:-video}
OUT=/rec/out; mkdir -p $OUT
source /opt/ros/jazzy/setup.bash; source /opt/px4_ws/install/setup.bash; source /ws/install/setup.bash
Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp >/dev/null 2>&1 &
Xvfb :98 -screen 0 960x540x24 -nolisten tcp >/dev/null 2>&1 &
sleep 2
ros2 launch drone_bringup sim.launch.py rviz:=false world:=$WORLD unknown_cost:=$COST \
  real_time_factor:=${RTF:-0.2} > $OUT/$NAME.log 2>&1 &
python3 /rec/world_markers.py /ws/src/drone_sim/worlds/$WORLD.sdf > /dev/null 2>&1 &
LP_NUM_THREADS=3 DISPLAY=:99 rviz2 -d /rec/video.rviz --fullscreen --ros-args -r __node:=rviz_top \
  -p use_sim_time:=true > $OUT/${NAME}_rviz_top.log 2>&1 &
LP_NUM_THREADS=2 DISPLAY=:98 rviz2 -d /rec/chase.rviz --fullscreen --ros-args -r __node:=rviz_chase \
  -p use_sim_time:=true > $OUT/${NAME}_rviz_chase.log 2>&1 &
(while true; do echo "$(date +%s) $(cut -d' ' -f1-3 /proc/loadavg)"; sleep 10; done) > $OUT/${NAME}_load.txt &
for i in $(seq 240); do grep -q "TAKEOFF -> HOVER" $OUT/$NAME.log && break; sleep 1; done
grep -q "TAKEOFF -> HOVER" $OUT/$NAME.log || { echo "NO HOVER"; exit 2; }
sleep 4
grab() {  # display size file
  ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -video_size $2 -i $1 -frames:v 1 $3
}
leg() {
  ros2 run drone_bringup mission_metrics.py $2 $3 --world $WORLD --timeout 900 \
    --label "$NAME leg $1" --csv $OUT/$NAME.csv 2>&1 | grep -E "reached|peak|clearance"
}
if [ "$MODE" = shot ]; then
  leg 1 19.5 0 &
  sleep ${SHOT_AFTER:-60}
  grab :99 1920x1080 $OUT/${NAME}_top.png; grab :98 960x540 $OUT/${NAME}_chase.png
  exit 0
fi
ENC="-c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p"
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 20 -video_size 1920x1080 -i :99 \
  $ENC $OUT/${NAME}_top.mp4 &
TOP=$!
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 15 -video_size 960x540 -i :98 \
  $ENC $OUT/${NAME}_chase.mp4 &
CHASE=$!
sleep 3
leg 1 19.5 0; sleep 4; leg 2 ${LEG2:-3.8 7.5}; sleep 4
kill -INT $TOP $CHASE; wait $TOP $CHASE
grep -c "Accel #0 fail" $OUT/$NAME.log | sed 's/^/IMU timeouts: /'
awk '{print $2}' $OUT/${NAME}_load.txt | sort -n | tail -1 | sed 's/^/peak 1-min load: /'
