/*
 *  DenseVisionClient.cpp
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

#include <dense_vision_client/DenseVisionClient.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


DenseVisionClient::DenseVisionClient(ros::NodeHandle nh_private) : nh_private_(nh_private)
{
    rgb_subscriber_ = nh_private_.subscribe("rgb", 1, &DenseVisionClient::topicCallbackRGB, this);
    disparity_subscriber_ = nh_private_.subscribe("disparity", 1, &DenseVisionClient::topicCallbackDisparity, this);
}

DenseVisionClient::~DenseVisionClient()
{

}

void DenseVisionClient::getROSParameters()
{
    // rate in Hz at which to communicate with DV server running on Aragorn
    nh_private_.param("dense_vision_comm_rate", dense_vision_comm_rate_, 30.0);

}

void DenseVisionClient::initConnection()
{
    boost::thread send_data_to_server_thread(boost::bind(&DenseVisionClient::sendDataToServer, this));
    send_data_to_server_thread.detach();
}


void DenseVisionClient::topicCallbackRGB(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr_rgb = cv_bridge::toCvShare(msg,
                                                                 sensor_msgs::image_encodings::BGR8);


    uchar *data_img;
    data_img = cv_ptr_rgb->image.data;

    const unsigned int ch=3;

    rgb_mutex_.lock();
    // copy to image buffer
    for(int i=0;i<480;i++)
    {
        for(int j=0;j<640;j++)
        {
            for(int k=0;k<ch;k++)
            {
                image_buffer_[ch*(i*640+j)+k] = data_img[ch*(i*640+j)+k];
            }
        }
    }
    rgb_mutex_.unlock();
}

void DenseVisionClient::topicCallbackDisparity(const stereo_msgs::DisparityImage::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(msg->image,
                                                             sensor_msgs::image_encodings::TYPE_32FC1);

    uchar *data_depth = cv_ptr_depth->image.data;

    depth_mutex_.lock();
    // copy to depth buffer
    for(int i=0;i<480;i++)
    {
        float *rowptr=(float*)(data_depth + i*640*sizeof(float));

        for(int j=0;j<640;j++)
        {
            depth_buffer_[i*640+j]=rowptr[j];
        }
    }
    depth_mutex_.unlock();
}

// sends the rgb + depth images to dense vision server via TCP
void DenseVisionClient::sendDataToServer()
{
    try
    {
        boost::asio::io_service io_service;
        boost::asio::ip::tcp::acceptor acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 22333));
        boost::asio::ip::tcp::socket socket(io_service);

        ros::Rate loop_rate(dense_vision_comm_rate_);

        while(nh_private_.ok())
        {
            acceptor.accept(socket);
            while(nh_private_.ok())
            {

                boost::array<char, 4> buf;
                boost::system::error_code error;
                size_t len = boost::asio::read(socket, boost::asio::buffer(buf),
                                               boost::asio::transfer_all(), error);
                if (error == boost::asio::error::eof)
                    break; // Connection closed cleanly by peer.
                else if (error)
                    throw boost::system::system_error(error); // Some other error.


                const unsigned int ch=3;


                boost::system::error_code ignored_error;

                rgb_mutex_.lock();
                boost::asio::write (socket, boost::asio::buffer (image_buffer_, 640*480*3*sizeof(uchar))  ,
                                    boost::asio::transfer_all(), ignored_error);
                rgb_mutex_.unlock();

                depth_mutex_.lock();
                boost::asio::write (socket, boost::asio::buffer (depth_buffer_, 640*480*sizeof(float)),
                                    boost::asio::transfer_all(), ignored_error);
                depth_mutex_.unlock();

                loop_rate.sleep();
            }
        }

    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        exit(1);
    }
}


