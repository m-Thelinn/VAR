
launch_points(){
    gnome-terminal -- bash -c "catkin_make && source devel/setup.bash && rosrun get_pointclouds get_pointclouds_node"
}

launch_points

