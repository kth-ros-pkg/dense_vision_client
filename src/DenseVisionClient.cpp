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

}

DenseVisionClient::~DenseVisionClient()
{

}

void DenseVisionClient::getROSParameters()
{
    // rate in Hz at which to communicate with DV server running on Aragorn
    nh_private_.param("dense_vision_comm_rate", dense_vision_comm_rate_, 30.0);

    // determine whether to use depth image or disparity image
    nh_private_.param("use_depth", use_depth_, false);

    // set the child frame ID of tracked object
    nh_private_.param("child_frame_id", child_frame_id_, std::string("DV_tracked_object"));
}

void DenseVisionClient::initTopicSub()
{
    rgb_subscriber_ = nh_private_.subscribe("rgb", 1, &DenseVisionClient::topicCallbackRGB, this);
    if(!use_depth_)
    {
        disparity_subscriber_ = nh_private_.subscribe("disparity", 1, &DenseVisionClient::topicCallbackDisparity, this);
    }
    else
    {
        depth_subscriber_ = nh_private_.subscribe("depth", 1, &DenseVisionClient::topicCallbackDepth, this);
    }

}

void DenseVisionClient::initConnection()
{
    // wait to obtain frame ID from RGB callback
    boost::unique_lock<boost::mutex> lock(frame_id_mutex_);
    while(frame_id_.empty())
    {
        frame_id_cond_.wait(lock);
    }

    boost::thread communicate_with_aragorn_thread(boost::bind(&DenseVisionClient::communicateWithAragorn, this));
    communicate_with_aragorn_thread.detach();
}


void DenseVisionClient::topicCallbackRGB(const sensor_msgs::Image::ConstPtr &msg)
{
    // set the frame id of the tracked object
    static bool frame_id_set = false;
    if(!frame_id_set)
    {
        boost::lock_guard<boost::mutex> lock(frame_id_mutex_);
        frame_id_ = msg->header.frame_id;
        frame_id_set = true;
        frame_id_cond_.notify_one();
    }

    cv_bridge::CvImageConstPtr cv_ptr_rgb;
    if(msg->encoding == "bgr8")
    {
        cv_ptr_rgb = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    else if(msg->encoding == "rgb8")
    {
        cv_ptr_rgb = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    }

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

void DenseVisionClient::topicCallbackDepth(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr_depth = cv_bridge::toCvShare(msg,
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

void DenseVisionClient::communicateWithAragorn()
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

                // get the pose and publish it to TF
                {
                    boost::array<double, 6> buf_pose;
                    boost::system::error_code error;
                    size_t len = boost::asio::read(socket, boost::asio::buffer(buf_pose),
                                                   boost::asio::transfer_all(), error);
                    if (error == boost::asio::error::eof)
                        break; // Connection closed cleanly by peer.
                    else if (error)
                        throw boost::system::system_error(error); // Some other error.

                    // Convert translation to meters and
                    // Transform the pose to right-hand system
                    for(unsigned int i=0; i<3; i++) buf_pose[i] = buf_pose[i]/1000.0;
                    buf_pose[1] = -buf_pose[1];

                    // transform the pose to StampedTransform format and publish it
                    tf::StampedTransform object_transform;
                    object_transform.setOrigin(tf::Vector3(buf_pose[0], buf_pose[1], buf_pose[2]));

                    tf::Quaternion q;
                    q.setRPY(buf_pose[3], buf_pose[4], buf_pose[5]);

                    object_transform.setRotation(q);
                    object_transform.stamp_ = ros::Time::now();
                    object_transform.child_frame_id_ = child_frame_id_;
                    object_transform.frame_id_ = frame_id_;

                    tfb_.sendTransform(object_transform);
                }


                // send the RGB + disparity images

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
        ROS_ERROR("Exception in Aragorn communication thread");
        std::cerr << e.what() << std::endl;
        exit(1);
    }
}


