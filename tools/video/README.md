# Demo video

Scripts that record and cut the demo video. Takes and cuts go to `out/` (git-ignored); the
finished video is published as a GitHub release asset, not committed.

- Recording image: `docker build -t drone-planner-rec:local .` here (drone-planner + xvfb, ffmpeg).
- Record a take: `./run_take.sh <world> <cost> <name>` (writes `out/`), e.g.
  `./run_take.sh factory_blocked 0.0 blocked_cost0`, or with the divided hall's return goal
  `LEG2="6 -7.5" ./run_take.sh factory_detour 1.0 detour_cost1`. It retries up to 3 times
  when PX4 doesn't take off. Each take is recorded at sim RTF 0.2: an
  overhead view (`<name>_top.mp4`) and a chase view (`<name>_chase.mp4`), plus the launch log.
  Check its output: IMU timeouts 0, peak speed ~1.3 m/s. `./run_take.sh ... test shot` grabs one
  frame of each view instead.
- Cut: with the takes in `out/`, run
  `docker run --rm -e RTF=0.2 -e OPEN_START=5.3 -e B0_START=277 -e B10_START=264 -e B0_TIME=78 -e B10_TIME=36 -e D1_START=186.9 -e D10_START=203.8 -e D1_TIME=26 -e D10_TIME=38 -v "$PWD:/rec" drone-planner-rec:local bash /rec/edit.sh`
  which writes `out/demo.mp4` (86 s): title, open hall at cost 1 (overhead + chase, 2x), card,
  blocked aisle return leg cost 0 vs 10 side by side (4x), card, divided hall return leg
  cost 1 vs 10 side by side (4x).
  Leg start offsets = planner "Goal (" log time - "TAKEOFF -> HOVER" time - 4 s.
- Load: two software-rendered RViz + capture ~13-15 load on 8 cores at RTF 0.2 (LP_NUM_THREADS
  capped). At RTF 0.5 the sim starved (IMU timeouts, crashes). Leg 1 still ~50 s vs 35-40 s in
  the benchmark.
