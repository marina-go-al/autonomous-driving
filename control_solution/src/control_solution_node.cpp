#include "../include/control_class.h"

int main(int argc, char **argv) {

  // Node "control_solution" initialization
  ros::init(argc, argv, "control_solution");

  // controlCar object of type ControlCar initialization
  // See control_class.cpp and control_class.h for more details on how the class has been programmed
  ControlCar controlCar;
  ros::spin();

  return 0;
}