/*
 * File: image_converter_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "image_converter.h"

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;

    ROS_INFO("image converter");

    // image converter node
    ImageConverter image_converter(nh);

    ros::spin();

    return 0;
}

