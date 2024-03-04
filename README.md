# VAR
- Miguel Rodríguez
- Santiago Nogales

## Practica 1
Durante esta practica realizadores el mapeo mediante un kinect en una habitacion, empleando ROS y algoritmos de nubes de puntos.

Para iniciar GAZEBO
- export TURTLEBOT3_MODEL=waffle
- catkin_make (compila los paquetes)
- source devel/setup.bash (dos veces si da error)
- roslaunch turtlebot_gazebo_multiple create_multi_robot.launch

Para el paquete de listener (otra terminal con el GAZEBO abierto):
- export TURTLEBOT3_MODEL=waffle
- source devel/setup.bash (dos veces si da error)
- rosrun listener listener