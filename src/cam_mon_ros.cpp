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
    init_chatter();

    timer = nh_.createTimer(ros::Duration(startup_delay), &Cam_mon::timer_cb, this, true);
  }

  void Cam_mon::init_chatter()
  {
    image_sub = nh_.subscribe("/usb_cam/image_raw", 1, &Cam_mon::cam_cb, this);
    ROS_INFO_STREAM("subscribe to the image_raw topic on [" << image_sub.getTopic() << "]");
  }

  void Cam_mon::init_params()
  {
    nh_.param("mon_node", global_config.mon_node, std::string("usb_cam"));
    nh_.param("launch_cmd", global_config.launch_cmd, std::string("roslaunch usb_cam usb_cam-test.launch"));
    nh_.param("topic_timeout", global_config.topic_timeout, 1.0);
    nh_.param("startup_delay", startup_delay, 10.0);
    nh_.param("restart_delay", global_config.restart_delay, 10.0);
    nh_.param("overexposure_threshold", global_config.overexposure_threshold, 245);
  }

  void Cam_mon::timer_cb(const ros::TimerEvent &event)
  {
    ROS_WARN("restart node!");
    restart_node();
  }

  void Cam_mon::cam_cb(const sensor_msgs::Image &msg)
  {
    // ROS_INFO_STREAM("image, received ");
    // reset timer
    timer.setPeriod(ros::Duration(global_config.topic_timeout), true);

    image_msg = msg;

    if (is_overexposed(image_msg))
    {
      ROS_WARN("The image is overexposed!");
      restart_node();
    }
  }

  // ----------------------------------------------------------------
  // HELPER FUNCTIONS **********************************************
  // ----------------------------------------------------------------
  void Cam_mon::shutdown()
  {
    ROS_WARN_STREAM("topic not published since [" << global_config.topic_timeout << "] seconds");
    ROS_WARN_STREAM("shutdown: [" << global_config.mon_node.c_str() << "]");
    std::string kill_cmd = std::string("rosnode kill /") + global_config.mon_node.c_str();
    system(kill_cmd.c_str());
  }

  void Cam_mon::startup()
  {
    std::string restart_cmd = std::string(global_config.launch_cmd.c_str()) + "&";
    ROS_WARN_STREAM("start node: " << restart_cmd.c_str());
    system(restart_cmd.c_str());
  }

  void Cam_mon::restart_node()
  {
    shutdown();
    ROS_WARN_STREAM("sleep for [" << global_config.restart_delay << "] seconds before starting the node again");
    sleep(global_config.restart_delay);

    startup();
    ROS_WARN_STREAM("sleep for [" << global_config.restart_delay << "] seconds to ensure node is up and running");
    sleep(global_config.restart_delay);
  }

  bool Cam_mon::is_overexposed(sensor_msgs::Image image_msg)
  {
    for (size_t i = 0; i < image_msg.data.size(); i++)
    {
      if (image_msg.data[i] < global_config.overexposure_threshold)
      {
        return false;
      }
    }
    return true;
  }
}  // namespace ros_cam_mon
