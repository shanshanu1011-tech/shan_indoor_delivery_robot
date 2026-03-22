#!/usr/bin/bash
gnome-terminal -- bash -c "source ~/delivery_robot/install/setup.bash; ros2 launch bot_description gazebo.launch.py" &
sleep 5
gnome-terminal -- bash -c "rviz2 -d ~/delivery_robot/src/bot_description/config/robot.rviz " &
sleep 5
gnome-terminal -- bash -c "source ~/delivery_robot/install/setup.bash;  ros2 launch bot_description online_async.launch.py " &
sleep 5
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true" &
sleep 5
gnome-terminal -- bash -c "source ~/delivery_robot/install/setup.bash; ros2 launch bot_description navigation.launch.py " &
sleep 5
