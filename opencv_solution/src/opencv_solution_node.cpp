#include "../include/opencv_class.h"

int main(int argc, char **argv) {

  // Node "opencv_solution" initialization
  ros::init(argc, argv, "opencv_solution");

  // detectPerson object of type DetectPerson initialization
  // See opencv_class.cpp and opencv_class.h for more details on how the class has been programmed
  DetectPerson detectPerson;
  ros::spin();

  return 0;
}