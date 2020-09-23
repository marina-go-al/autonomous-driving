#include "../include/pcl_class.h"

int main(int argc, char **argv) {

  // Node "pcl_solution" initialization
  ros::init(argc, argv, "pcl_solution");

  // detectObstacle object of type DetectObstacle initialization
  // See pcl_class.cpp and pcl_class.h for more details on how the class has been programmed
  DetectObstacle detectObstacle;
  ros::spin();

  return 0;
}