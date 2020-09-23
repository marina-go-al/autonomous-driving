#ifndef CONTROL_CLASS_H
#define CONTROL_CLASS_H

#include <algorithm>
#include <iostream>
#include <map>
#include <prius_msgs/Control.h>
#include <ros/ros.h>
#include <vector>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>

// ControlCar class which controls the movement of the car based on the
// OpenCV and PCL messages
class ControlCar {

private:
  // ROS Objects
  ros::NodeHandle nh_;

  // ROS Publishers
  ros::Publisher pub;

  // ROS Subscribers
  ros::Subscriber sub_openCV;
  ros::Subscriber sub_PCL;

  // ROS Messages
  prius_msgs::Control prius_msg;

  // Person Detected Attribute
  bool valid_person;

  // Callback functions encapsulation
  void callback_openCV(const vision_msgs::Detection2DArrayPtr &msg);
  void callback_PCL(const vision_msgs::Detection3DArrayPtr &msg);

public:
  // Default Constructor
  ControlCar();
};

#endif