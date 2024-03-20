
launch_catkin(){
    gnome-terminal -- bash -c "export TURTLEBOT3_MODEL=waffle && catkin_make && source devel/setup.bash && roslaunch turtlebot_gazebo_multiple create_multi_robot.launch"
}

launch_movement(){
    gnome-terminal -- bash -c "catkin_make && source devel/setup.bash && rosrun movement movement_node"
}

launch_points(){
    gnome-terminal -- bash -c "catkin_make && source devel/setup.bash && rosrun get_pointclouds get_pointclouds_node"
}

launch_catkin

