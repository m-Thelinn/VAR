
launch_movement(){
    gnome-terminal -- bash -c "catkin_make && source devel/setup.bash && rosrun movement movement_node"
}

launch_movement