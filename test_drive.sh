#!/usr/bin/env bash
#
# test_rt_cmd_sequence.sh — Drive StoRPer vehicle using Twist messages on /rt/cmd_vel
# Sequence:
#   1. Forward at 50% for 5s
#   2. Reverse at 20% for 5s
#   3. Forward + turn right for 5s
#   4. Reverse + turn left for 5s

IMAGE=ros:kilted-ros-base

echo "⏳ Pulling Docker image if needed..."
docker pull $IMAGE > /dev/null

docker run --rm -it \
  --network host \
  -v "$PWD":/workspace -w /workspace \
  $IMAGE \
  bash -lc "
    source /opt/ros/kilted/setup.bash

    echo; echo '→ 1. FORWARD (50%) for 5s'
    ros2 topic pub /rt/cmd_vel geometry_msgs/msg/Twist \
      \"{linear: {x: 0.25}, angular: {z: 0.25}}\" \
      --rate 2 --times 10

    echo; echo '→ 2. REVERSE (20%) for 5s'
    ros2 topic pub /rt/cmd_vel geometry_msgs/msg/Twist \
      \"{linear: {x: -0.25}, angular: {z: -0.25}}\" \
      --rate 2 --times 10

    echo; echo '→ 3. FORWARD + RIGHT TURN for 5s'
    ros2 topic pub /rt/cmd_vel geometry_msgs/msg/Twist \
      \"{linear: {x: 0.7}, angular: {z: -0.7}}\" \
      --rate 2 --times 10

    echo; echo '→ 4. REVERSE + LEFT TURN for 5s'
    ros2 topic pub /rt/cmd_vel geometry_msgs/msg/Twist \
      \"{linear: {x: -0.7}, angular: {z: 0.7}}\" \
      --rate 2 --times 10

    echo; echo '→ STOP'
    ros2 topic pub --once /rt/cmd_vel geometry_msgs/msg/Twist \
      \"{linear: {x: 0.0}, angular: {z: 0.0}}\"
  "

