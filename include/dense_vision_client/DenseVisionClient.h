/*
 *  DenseVisionClient.h
 *
 *
 *  Created on: Aug 15, 2014
 *  Authors:   Francisco Viña          Yasemin Bekiroglu
 *            fevb <at> kth.se        yaseminb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DENSE_VISION_CLIENT_H_
#define DENSE_VISION_CLIENT_H_

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

class DenseVisionClient
{
public:

    DenseVisionClient(ros::NodeHandle nh_private);

    virtual ~DenseVisionClient();

    void getROSParameters();

    void initConnection();

    void topicCallbackRGB(const sensor_msgs::Image::ConstPtr &msg);

    void topicCallbackDisparity(const stereo_msgs::DisparityImage::ConstPtr &msg);

    void topicCallbackDepth(const sensor_msgs::Image::ConstPtr &msg);

    // sends the rgb + depth images to dense vision server via TCP
    void sendDataToServer();


private:
    ros::NodeHandle nh_private_;

    ros::Subscriber rgb_subscriber_;
    ros::Subscriber disparity_subscriber_;

    boost::array<char,640*480*3> image_buffer_;
    boost::array<float,640*480> depth_buffer_;

    boost::mutex rgb_mutex_;
    boost::mutex depth_mutex_;

    double dense_vision_comm_rate_;
};

#endif
