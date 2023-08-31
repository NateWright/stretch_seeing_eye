# stretch_seeing_eye

[AWS Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world)

# Gazebo Sim With Map
```bash
roslaunch stretch_seeing_eye gazebo.launch
roslaunch stretch_seeing_eye navigation_gazebo.launch location:=aws_hospital
roslaunch stretch_seeing_eye navigate_waypoints.launch location:=aws_hospital
roslaunch rosbridge_server rosbridge_websocket.launch
```
# Real Robot Generating Map
```bash
# On Robot
roslaunch stretch_seeing_eye stretch_remote_bringup.launch
roslaunch stretch_seeing_eye mapping.launch rviz:=false teleop_type:=none
# On terminal
rviz -d `rospack find stretch_navigation`/rviz/mapping.rviz
roslaunch stretch_core teleop_twist.launch teleop_type:=keyboard
rosrun map_server map_saver
```

# Real Robot with Map
```bash
# On Robot
roslaunch stretch_seeing_eye stretch_remote_bringup.launch
roslaunch stretch_seeing_eye navigation.launch location:=dan rviz:=false
roslaunch face_detector face_detector.rgbd.launch rgb_ns:=color image_topic:=image_raw depth_ns:=aligned_depth_to_color fixed_frame:=base_link depth_topic:=image_raw
roslaunch stretch_seeing_eye navigate_waypoints.launch location:=dan
roslaunch rosbridge_server rosbridge_websocket.launch
# On terminal
rviz -d `rospack find stretch_seeing_eye`/rviz/navigation.rviz

# Face Detector test
```

```bash
roslaunch stretch_seeing_eye navigation.launch location:=gps_room rviz:=false
rviz -d `rospack find stretch_seeing_eye`/rviz/navigation.rviz
```

```bash
rosrun rosserial_python serial_node.py /dev/stretch-handle
```

udev rules
```txt
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2a03", GROUP="plugdev", MODE="0666", SYMLINK+="stretch-handle"
```
