# cam_mon

## Contents

- [Run](#run)
- [Requirenments](#requirenments) 
<!-- - [Published Topics](#published-topics) -->
- [Subscribed Topics](#subscribed-topics)
- [Parameters](#parameters)
- [Development](#development)
  - [Debugging C++](#debugging-c)
  - [Linting](#linting)
- [TODO](#todo)

Monitor a realsense camera node.
The realsense camera frequemtly keeps overexposing the image during startup. To emitigate this issue a simple disable and enable service call can reduce this issue successfuly. The node subscribe to the image topic. Once a image is received the camera will be disabled and reenabled. Afterwards the node has done it's part and shuts down.

## Requirenments 
Only works with realsense camera.
    

## Run

    roslaunch cam_mon cam_mon.launch 

<!-- ## Published Topics

- chatter [std_msgs/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html) -->

## Subscribed Topics

- /usb_cam/image_raw [(sensor_msgs/Image)](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

## Parameters

- enable_cmd (string,default:"rosservice call /cameraF_down/enable 'data: true'")
  - cmd to enable realsense camera
- disable_cmd (string,default:"rosservice call /cameraF_down/enable 'data: false'")
  - cmd to disable realsense camera
- launch_cmd (string,default:"roslaunch bringup rs_cameraF_down.launch")
  - launch camera if there's no topic published
- restart_delay (double,default: 10.0)
  - wait time in seconds before reenabeling the realsense cam

Demo graph for example
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
