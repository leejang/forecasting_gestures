/*
 * Cloud Robotics Project with ROS
 * File: image_converter.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "image_converter.h"
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//static const std::string OPENCV_WINDOW = "Turtlebot Image";

ImageConverter::ImageConverter(ros::NodeHandle nh)
{
    this->nh = nh;
    it = new image_transport::ImageTransport(nh);
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it->subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

    //cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter()
{
    if (it != NULL) delete it;
    //cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    std::stringstream sstream;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(1);

    sstream << "/home/leejang/ros_ws/src/forecasting_gestures/cur_image/cur_image.jpg";
    ROS_ASSERT( cv::imwrite(sstream.str(), cv_ptr->image));

}

