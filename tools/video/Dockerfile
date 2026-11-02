FROM drone-planner:latest
USER root
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends xvfb ffmpeg && rm -rf /var/lib/apt/lists/*
USER ubuntu
