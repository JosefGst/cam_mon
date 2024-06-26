# cam_mon

## Contents

- [Run](#run)
<!-- - [Setup](#setup)  -->
<!-- - [Published Topics](#published-topics) -->
- [Subscribed Topics](#subscribed-topics)
- [Parameters](#parameters)
- [Development](#development)
  - [Debugging C++](#debugging-c)
  - [Linting](#linting)
- [TODO](#todo)

Monitor a camera node.
Sometimes the ROS node is running without showing any errors,  however no topics are published. In case the camera is not publishing or the image is overexposed then it will restart the node. 

## Setup 
(Optional) For the demo to work install.

    sudo apt install ros-noetic-usb-cam
    

## Run

    roslaunch cam_mon cam_mon.launch 

<!-- ## Published Topics

- chatter [std_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) -->

## Subscribed Topics

- /usb_cam/image_raw [(sensor_msgs/Image)](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

## Parameters

- mon_node (string,default: "usb_cam")
  - Node to monitor
- launch_cmd (string,default:"roslaunch usb_cam usb_cam-test.launch")
  - cmd to restart the node
- topic_timeout (double,default: 1.0)
  - if no topic received during duration
- startup_delay (double,default: 10.0)
  - wait time in seconds until node is fully started before monitoring
- restart_delay (double,default: 10.0)
  - the shutdown takes time, wait time in seconds before restarting node
- overexposure_threshold (int,default 245)
  - theshold for the overexposure. If value is lower it is more sensible. (0 - 255)

![graph](assets/rosgraph.svg)

## Development

### Debugging C++

1. Move the .vscode folder into the workspace directory. 
2. Edit the launch.json file as necessary.
3. Run the "make_debug" task
4. Click the Debug button on the left side in vscode. Select "ROS:Launch" and click the the green arrow.

#### Alternatively attach to single node

In addition to the previous steps 
1. Run the "ROS:Start" task
2. rosrun the node
3. Click the green debug button with ROS:attach selected

#### References
[Polyhobbyist Youtube](https://www.youtube.com/watch?v=uqqHgYsskJI)
[vscode-ros github](https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.mdA)

### Linting

In root of workspace

    catkin_make roslint_cam_mon

## TODO

- [ ] publish cmd_vel topic to stop the robot while restarting node