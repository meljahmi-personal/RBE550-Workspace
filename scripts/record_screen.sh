#!/bin/bash

# Set output path
OUTPUT_FILE="$HOME/ros2projrbe550_ws/video/multiple_algorithms.mkv"

# Create video directory if it doesn't exist
mkdir -p "$(dirname "$OUTPUT_FILE")"

# Record screen for 5 minutes (300 seconds)
ffmpeg -f x11grab -video_size 1920x1080 -framerate 30 -i :1 \
       -f pulse -i default \
       -c:v libx264 -c:a copy \
       -t 300 \
       "$OUTPUT_FILE"

# Notify when done
notify-send "Screen Recording" "Recording saved to $OUTPUT_FILE"
