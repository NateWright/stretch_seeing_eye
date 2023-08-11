# stretch_seeing_eye

[AWS Hospital](https://github.com/aws-robotics/aws-robomaker-hospital-world)

```bash
roslaunch stretch_seeing_eye gazebo.launch
roslaunch stretch_seeing_eye navigation_gazebo.launch location:=aws_hospital
roslaunch stretch_seeing_eye navigate_waypoints.launch location:=aws_hospital
roslaunch stretch_seeing_eye feature_detection.launch location:=aws_hospital
roslaunch rosbridge_server rosbridge_websocket.launch
```

```bash
roslaunch stretch_core stretch_remote_bringup.launch
roslaunch stretch_seeing_eye navigation.launch location:=dan rviz:=false
roslaunch stretch_seeing_eye navigate_waypoints.launch location:=dan
rviz -d `rospack find stretch_seeing_eye`/rviz/navigation.rviz
roslaunch rosbridge_server rosbridge_websocket.launch
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
