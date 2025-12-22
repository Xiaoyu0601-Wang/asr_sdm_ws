#!/bin/bash
# Test script for EuRoC dataset with SVO

cd ~/svo/asr_sdm_ws
source install/setup.bash

echo "=== Starting SVO node for EuRoC dataset ==="
ros2 launch svo_ros test_euroc.launch.py &
SVO_PID=$!

echo "Waiting 3 seconds for node to initialize..."
sleep 3

echo "=== Playing MH_01_easy rosbag at 0.5x speed ==="
ros2 bag play datasheet/MH_01_easy_ros2 -r 0.5 --clock 200

echo "=== Test complete, stopping SVO node ==="
kill $SVO_PID
wait $SVO_PID 2>/dev/null

echo "=== Done ==="

