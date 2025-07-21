#!/usr/bin/env bash
#
# test_rt/cmd_vel.sh — send forward/back/left/right Twist on /rt/cmd_vel
# Usage: ./test_rt/cmd_vel.sh

# Which Docker image to use
IMAGE=ros:kilted-ros-base   # ROS 2 Kilted (ros:rolling-ros-base also works) :contentReference[oaicite:0]{index=0}

# Make sure image is present
docker pull $IMAGE

# Run the container, using host networking so UDP packets hit the micro-ROS agent
docker run --rm -it \
  --network host \
  -v "$PWD":/workspace -w /workspace \
  $IMAGE \
  bash -lc "
    source /opt/ros/kilted/setup.bash

    echo; echo '→ FORWARD'; 
    ros2 topic pub --once /rt/cmd_vel geometry_msgs/Twist \
      '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'

    sleep 1
    echo; echo '→ BACKWARD';
    ros2 topic pub --once /rt/cmd_vel geometry_msgs/Twist \
      '{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.2}}'

    sleep 1
    echo; echo '→ LEFT';
    ros2 topic pub --once /rt/cmd_vel geometry_msgs/Twist \
      '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

    sleep 1
    echo; echo '→ RIGHT';
    ros2 topic pub --once /rt/cmd_vel geometry_msgs/Twist \
      '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}'

    sleep 1
    echo; echo '→ Done.'
    ros2 topic pub --once /rt/cmd_vel geometry_msgs/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
  "

