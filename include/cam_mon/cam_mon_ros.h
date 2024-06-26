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

#ifndef CAM_MON_ROS_H
#define CAM_MON_ROS_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

namespace ros_cam_mon
{
class Cam_mon
{
public:
    Cam_mon();
    void init_chatter();

private:
    ros::NodeHandle nh_;

    void init_params();
    void cam_cb(const sensor_msgs::Image &msg);
    
    /**
     * @brief Timer Callback function.
     *
     * Main Function which is frequently called by the timer.
     *
     */
    void timer_cb(const ros::TimerEvent &event);
    void shutdown();
    void startup();
    void restart_node();
    ros::Timer timer;

    std_msgs::String string_msg;
    sensor_msgs::Image image_msg;
    ros::Publisher chatter_pub;

    ros::Subscriber image_sub;

    // PARAMS
    int rate = 1;
};
}  // namespace ros_cam_mon

#endif  // CAM_MON_ROS_H
