#!/bin/bash

rm -rf src/ 
mkdir src/
cd src

# Creating all packages
ros2 pkg create hardware --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2 --dependencies rclcpp std_msgs
ros2 pkg create communication --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2 --dependencies rclcpp std_msgs
ros2 pkg create master --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2 --dependencies rclcpp std_msgs
ros2 pkg create vision --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2 --dependencies rclcpp std_msgs
ros2 pkg create world_model --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2 --dependencies rclcpp std_msgs
ros2 pkg create ros2_utils --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2 --dependencies rclcpp std_msgs
ros2 pkg create ros2_interface --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2
ros2 pkg create web_ui --build-type ament_cmake --license MIT --maintainer-name $1 --maintainer-email $2

