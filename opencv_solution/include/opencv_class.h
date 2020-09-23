#ifndef OPENCV_CLASS_H
#define OPENCV_CLASS_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>

// DetectPerson class declaration for the OpenCV person
// detection algorithm
class DetectPerson {

private:
  // ROS Objects
  ros::NodeHandle nh_;

  // ROS Publishers
  ros::Publisher pub_detections;
  ros::Publisher pub_visual;

  // ROS Subscribers
  ros::Subscriber sub;

  // Callback encapsulation
  void callbackConversion(const sensor_msgs::ImageConstPtr &msg);

public:
  // Default Constructor
  DetectPerson();
};


#endif