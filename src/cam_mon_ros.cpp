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


    sleep(global_config.startup_delay);
    timer = nh_.createTimer(ros::Duration(1), &Cam_mon::timer_cb, this, true);
  }

  void Cam_mon::init_chatter()
  {
    chatter_pub = nh_.advertise<std_msgs::String>("/chatter", 1);
    ROS_INFO_STREAM("advertise to chatter topic on [" << chatter_pub.getTopic() << "]");

    image_sub = nh_.subscribe("/usb_cam/image_raw", 1, &Cam_mon::cam_cb, this);
    ROS_INFO_STREAM("subscribe to the image_raw topic on [" << image_sub.getTopic() << "]");
  }

  void Cam_mon::init_params()
  {
    if (!nh_.param("rate", rate, 10))
    {
      ROS_WARN("No rate set. Default is 10");
    }
    if (!nh_.param("pub_string", global_config.pub_string, std::string("Hello World!")))
    {
      ROS_WARN("No pub_string set. Default is 'Hello World!'");
    }
  }

  void Cam_mon::timer_cb(const ros::TimerEvent &event)
  {
    string_msg.data = global_config.pub_string;
    chatter_pub.publish(string_msg);
    shutdown();
    sleep(global_config.restart_delay);
    
    restart();
    sleep(global_config.restart_delay);
  }

  void Cam_mon::cam_cb(const sensor_msgs::Image &msg)
  {
    // ROS_INFO_STREAM("image, received ");
    // reset timer
    timer.setPeriod(ros::Duration(global_config.topic_timeout), true);

    
  }

  void Cam_mon::shutdown()
  {
    ROS_WARN_STREAM("topic not published since [" << global_config.topic_timeout << "] seconds");
    ROS_WARN_STREAM("shutdown: [" << global_config.mon_node.c_str() << "]");
    std::string kill_cmd = std::string("rosnode kill /") + global_config.mon_node.c_str();
    system(kill_cmd.c_str());
  }

  void Cam_mon::restart()
  {
    std::string restart_cmd = std::string(global_config.launch_cmd.c_str()) + "&";
    ROS_WARN_STREAM("restart node: " << restart_cmd.c_str());
    system(restart_cmd.c_str());
  }

}  // namespace ros_cam_mon
