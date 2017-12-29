#pragma once
/*
 * Cloud Robotics Project with ROS
 * File: image_converter.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef IMG_CONVT_H_
#define IMG_CONVT_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class ImageConverter
{
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_sub_;

    void imageCb(const sensor_msgs::ImageConstPtr& msg);
  
public:
    ImageConverter(ros::NodeHandle nh);
    ~ImageConverter();
};

#endif /* IMG_CONVT_H_ */
