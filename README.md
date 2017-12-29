# forecasting_gestures

## How to run

``` bash
// 0) roscore -- bring up turtlebot on turtlebot laptop
roslaunch turtlebot_bringup minimal.launch --screen

// 1) run usb cam ros device driver on turtlebot laptop
rosrun usb_cam usb_cam_node

// 2) launch turtlebot teleoperation on turtlebot laptop
roslaunch turtlebot_teleop keyboard_teleop.launch --screen
```
