#!/bin/bash

# Launch RVIZ and gazebo and load up the map and world
ros2 launch petsitter_nav2_system my_tb3_real_nav2.launch.py &
#pid[0]=$!

# Check for a running RVIZ instance
while ! pgrep "rviz2" >/dev/null; do
    sleep 4
done

# Run the publisher script
ros2 run petsitter_nav2_system initial_pose_pub
if [$? -eq 0]
then
    echo "Published robot's initial position."
else
    echo "Something went wrong. Failed to publish the robot's initial position."
fi

# SIGNINT kills background RVIZ and Gazebo processes
trap "kill rviz2 gzclient gzserver"; exit 1 INT
wait