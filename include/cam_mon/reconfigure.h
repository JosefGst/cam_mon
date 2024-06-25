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

#ifndef CAM_MON_RECONFIGURE_H
#define CAM_MON_RECONFIGURE_H

#include <dynamic_reconfigure/server.h>
#include <cam_mon/ReconfigureConfig.h>

cam_mon::ReconfigureConfig global_config;

void reconfigure_cb(cam_mon::ReconfigureConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %s, %f",
           config.pub_string.c_str(),
           config.topic_timeout);
  global_config = config;
}

#endif  // CAM_MON_RECONFIGURE_H
