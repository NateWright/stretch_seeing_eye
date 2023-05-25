# stretch_seeing_eye

[AWS Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world)

```bash
roslaunch stretch_seeing_eye gazebo.launch
roslaunch stretch_seeing_eye navigation_gazebo.launch map_yaml:=/home/nwright/ros/catkin_ws/src/stretch_seeing_eye/stretch_seeing_eye/maps/aws_hospital/map.yaml
roslaunch stretch_seeing_eye navigate_waypoints.launch location:=aws_hospital
roslaunch stretch_seeing_eye feature_detection.launch location:=aws_hospital
```