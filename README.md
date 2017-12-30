# forecasting_gestures

## How to run

``` bash
// 0) roscore -- bring up turtlebot on turtlebot laptop
roslaunch turtlebot_bringup minimal.launch --screen

// 1) run usb cam ros device driver on turtlebot laptop
rosrun usb_cam usb_cam_node

// 2) run sound play node on turtlebot laptop
rosrun sound_play soundplay_node.py

// 3) run turtlebot control node on Madthunder
// in forecasting_gesture directory
python script/turtlebot_control_node.py

// 4) run geture forecasting node on Madthunder
// in forecasting_gesture directory
python script/gesture_forecasting_node.py
```
