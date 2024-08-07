/*
 * MIT License
 *
 * Copyright (c) 2024 Josef Gstoettner
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ros/ros.h"
#include "cam_mon/cam_mon_ros.h"
#include "cam_mon/reconfigure.h"
#include <string>

namespace ros_cam_mon
{

  Cam_mon::Cam_mon() : nh_("~")
  {
    init_params();
    init_monitor();
    timer = nh_.createTimer(ros::Duration(1.0/rate), &Cam_mon::timer_cb, this, oneshot, autostart_off);
    sm_timer = nh_.createTimer(ros::Duration(1.0), &Cam_mon::sm_timer_cb, this);
  }

  void Cam_mon::init_monitor()
  {
    image_sub = nh_.subscribe("/usb_cam/image_raw", 1, &Cam_mon::cam_cb, this);
    ROS_INFO_STREAM("subscribe to the image_raw topic on [" << image_sub.getTopic() << "]");
  }

  void Cam_mon::init_params()
  {
    nh_.param("enable_cmd", global_config.enable_cmd, std::string("rosservice call /cameraF_down/enable 'data: true'"));
    nh_.param("disable_cmd", global_config.disable_cmd, std::string("rosservice call /cameraF_down/enable 'data: false' "));
    nh_.param("launch_cmd", global_config.launch_cmd, std::string("roslaunch bringup rs_cameraF_down.launch"));
    nh_.param("restart_delay", global_config.restart_delay, 5.0);
  }

  void Cam_mon::cam_cb(const sensor_msgs::Image &msg)
  {
    if (state == 0){
      restart_node();
      state = 1;
      timer.start();
    }

    if (state == 1){
      timer.setPeriod(ros::Duration(1.0/rate), timer_reset);
    }
  }

  void Cam_mon::timer_cb(const ros::TimerEvent & event)
  {
    ROS_WARN("NO TOPIC RECEIVED");
    state = 0;
  }

  void Cam_mon::sm_timer_cb(const ros::TimerEvent & event)
  {
    if (state == 0){
      state_zero_counter++;
    }
    else{
      state_zero_counter = 0;
    }
    if(state_zero_counter > 20){
      ROS_WARN("launch cam node");
      launch_cmd();
      state_zero_counter = 0;
    }
    // ROS_WARN_STREAM("Debugg Timer state: " << state);
  }
  // ----------------------------------------------------------------
  // HELPER FUNCTIONS **********************************************
  // ----------------------------------------------------------------

  void Cam_mon::launch_cmd()
  {
    std::string launch_cmd = std::string(global_config.launch_cmd.c_str()) + "&";
    ROS_WARN_STREAM("launch node: " << launch_cmd.c_str());
    system(launch_cmd.c_str());
  }

  void Cam_mon::disable()
  {
    std::string disable_cmd = std::string(global_config.disable_cmd.c_str()) + "&";
    ROS_WARN_STREAM("disable: [" << disable_cmd.c_str() << "]");
    system(disable_cmd.c_str());
  }

  void Cam_mon::enable()
  {
    std::string enable_cmd = std::string(global_config.enable_cmd.c_str()) + "&";
    ROS_WARN_STREAM("start node: " << enable_cmd.c_str());
    system(enable_cmd.c_str());
  }

  void Cam_mon::restart_node()
  {
    disable();
    ROS_WARN_STREAM("sleep for [" << global_config.restart_delay << "] seconds before starting the node again");
    sleep(global_config.restart_delay);

    enable();
    ROS_WARN_STREAM("sleep for [" << global_config.restart_delay << "] seconds to ensure node is up and running");
    sleep(global_config.restart_delay);
  }

}  // namespace ros_cam_mon
