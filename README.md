# stretch_seeing_eye

[AWS Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world)
roslaunch stretch_seeing_eye gazebo.launch
roslaunch stretch_navigation navigation_gazebo.launch map_yaml:=/home/nwright/ros/catkin_ws/src/stretch_seeing_eye/stretch_seeing_eye/maps/aws_hospital/map.yaml

rosparam set /waypoints_file '/home/nwright/ros/catkin_ws/src/stretch_seeing_eye/stretch_seeing_eye/config/aws_hospital_wapoints/waypoints.csv'


roslaunch stretch_navigation navigation_gazebo.launch map_yaml:=/home/nwright/ros/catkin_ws/src/stretch_seeing_eye/stretch_seeing_eye/maps/gps_room/map.yaml