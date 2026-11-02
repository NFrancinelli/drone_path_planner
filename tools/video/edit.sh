#!/usr/bin/env bash
# 1920x1080 cut from the takes in /rec/out -> /rec/out/demo.mp4. Raw takes are at sim RTF
# $RTF (0.2): playing them N times faster shows N * RTF times real time.
# Env: OPEN_START, B0_START, B10_START, D1_START, D10_START (s into the raw takes), B0_TIME,
# B10_TIME, D1_TIME, D10_TIME (leg 2, s).
set -e
cd /rec/out
F=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
B=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
BG=0x26292f
CROP="crop=1260:1080:330:0"  # the hall in the overhead view
ENC="-c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p -r 30"
# text, size, x, y, font
t() { echo "drawtext=fontfile=${5:-$F}:text='$1':fontsize=$2:fontcolor=white:x=$3:y=$4"; }
c() { echo "drawtext=fontfile=${5:-$F}:text='$1':fontsize=$2:fontcolor=white:x=(w-text_w)/2:y=$3"; }
OPEN_X=$(awk "BEGIN{print 2 / $RTF}")   # 2x real time
BLOCK_X=$(awk "BEGIN{print 4 / $RTF}")  # 4x real time

# 1. Title
ffmpeg -loglevel error -y -f lavfi -i color=c=$BG:s=1920x1080:d=4.5:r=30 -vf "\
$(c '3D path planning on a live OctoMap' 68 400 $B),\
$(c 'A* + trajectory smoothing + replanning  |  PX4 SITL in Gazebo  |  ROS 2' 34 510),\
$(c 'The drone only knows what its depth camera has seen' 34 565)" $ENC t1.mp4

# 2. Open hall: overhead (cropped) left, chase view and captions right, 2x real time
ffmpeg -loglevel error -y -ss $OPEN_START -i open_cost1_top.mp4 -ss $OPEN_START -i open_cost1_chase.mp4 \
  -filter_complex "\
[0:v]setpts=PTS/$OPEN_X,fps=30,$CROP[top];\
[1:v]setpts=PTS/$OPEN_X,fps=30,scale=640:360[chase];\
color=c=$BG:s=660x1080:r=30[side];\
[side][chase]overlay=10:110:shortest=1,\
$(t 'Open hall' 40 20 24 $B),\
$(t 'unknown_cost 1 (default)  |  2x real time' 24 20 70),\
$(t 'Chase view' 22 20 480 $B),\
$(t 'Faint boxes\: the real walls' 24 20 560),\
$(t 'Blue\: what the camera has mapped' 24 20 600),\
$(t 'Yellow\: the drone' 24 20 640),\
$(t 'Magenta\: A* path' 24 20 680),\
$(t 'Orange\: the smoothed trajectory it flies' 24 20 720),\
$(t 'Green\: where it has flown' 24 20 760),\
$(t 'Red sphere\: where a plan got blocked' 24 20 800),\
$(t 'Leg 1\: to the goal behind the crate wall.' 24 20 870),\
$(t 'The corridor dead-ends\: it climbs over.' 24 20 910),\
$(t 'Leg 2\: back to the start area.' 24 20 960)[right];\
[top][right]hstack=inputs=2:shortest=1[v]" -map "[v]" -an $ENC t2.mp4

# 3. Section card
ffmpeg -loglevel error -y -f lavfi -i color=c=$BG:s=1920x1080:d=4:r=30 -vf "\
$(c 'Same hall, north aisle closed' 64 430 $B),\
$(c 'Return leg\: gamble on the unexplored aisle, or fly back the way it knows?' 34 530)" $ENC t3.mp4

# 4. Blocked aisle, return leg, overhead side by side, 4x real time
ffmpeg -loglevel error -y -ss $B0_START -i blocked_cost0_top.mp4 -ss $B10_START -i blocked_cost10_top.mp4 \
  -filter_complex "\
[0:v]setpts=PTS/$BLOCK_X,fps=30,$CROP,scale=940:806[a];\
[1:v]setpts=PTS/$BLOCK_X,fps=30,$CROP,scale=940:806,tpad=stop_mode=clone:stop_duration=120[b];\
color=c=$BG:s=1920x1080:r=30[bg];\
[bg][a]overlay=13:150:shortest=1[x];[x][b]overlay=967:150:shortest=1,\
$(c 'Blocked aisle  |  return leg  |  4x real time' 44 40 $B),\
drawtext=fontfile=$B:text='unknown_cost 0\: tries the aisle, goes round  (${B0_TIME} s)':fontsize=28:fontcolor=white:x=13+(940-text_w)/2:y=105,\
drawtext=fontfile=$B:text='unknown_cost 10\: the known corridor  (${B10_TIME} s)':fontsize=28:fontcolor=white:x=967+(940-text_w)/2:y=105,\
$(c 'Benchmark, 3 runs each\: 79 s vs 36 s. All 18 legs reached the goal, none closer than 0.49 m to an obstacle.' 26 990)[v]" \
  -map "[v]" -an $ENC t4.mp4

# 5. Section card
ffmpeg -loglevel error -y -f lavfi -i color=c=$BG:s=1920x1080:d=4:r=30 -vf "\
$(c 'A hall cut by a divider, a door at each end' 64 430 $B),\
$(c 'It sees the north door on the way out, never the south one. Same question on the way back.' 34 530)" $ENC t5.mp4

# 6. Divided hall, return leg, overhead side by side, 4x real time
ffmpeg -loglevel error -y -ss $D1_START -i detour_cost1_top.mp4 -ss $D10_START -i detour_cost10_top.mp4 \
  -filter_complex "\
[0:v]setpts=PTS/$BLOCK_X,fps=30,$CROP,scale=940:806,tpad=stop_mode=clone:stop_duration=120[a];\
[1:v]setpts=PTS/$BLOCK_X,fps=30,$CROP,scale=940:806[b];\
color=c=$BG:s=1920x1080:r=30[bg];\
[bg][b]overlay=967:150:shortest=1[x];[x][a]overlay=13:150:shortest=1,\
$(c 'Divided hall  |  return leg  |  4x real time' 44 40 $B),\
drawtext=fontfile=$B:text='unknown_cost 1\: finds the unseen door  (${D1_TIME} s)':fontsize=28:fontcolor=white:x=13+(940-text_w)/2:y=105,\
drawtext=fontfile=$B:text='unknown_cost 10\: back the way it knows  (${D10_TIME} s)':fontsize=28:fontcolor=white:x=967+(940-text_w)/2:y=105,\
$(c 'Benchmark, 3 runs each\: 26 s vs 41 s. The cost is a bet\: it wins when the unknown is a dead end, loses when it is a shortcut.' 26 990)[v]" \
  -map "[v]" -an $ENC t6.mp4

printf "file t1.mp4\nfile t2.mp4\nfile t3.mp4\nfile t4.mp4\nfile t5.mp4\nfile t6.mp4\n" > list.txt
ffmpeg -loglevel error -y -f concat -safe 0 -i list.txt -c copy demo.mp4
for f in t1 t2 t3 t4 t5 t6 demo; do printf "%s %s\n" $f "$(ffprobe -v error -show_entries format=duration -of csv=p=0 $f.mp4)"; done
