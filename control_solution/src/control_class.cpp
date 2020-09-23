#include "../include/control_class.h"

using namespace std;

// callback_openCV: 1) Gets the total number of detected persons with the camera
//                  2) Loops through all the detections (in case that there is one or more)
//                  3) If size > 25000, the function sets the attribute valid_person to "false"
//                  4) Otherwise, valid_person = "true"
void ControlCar::callback_openCV(const vision_msgs::Detection2DArrayPtr &msg) {

    int total_detections = (*msg).detections.size();

    if (total_detections != 0) {

        for (int i = 0; i < total_detections; i++) {

        vision_msgs::Detection2D aux = (*msg).detections[i]; // Auxiliar array that stores each detection

        if (aux.bbox.size_x * aux.bbox.size_y > 25000) {
            valid_person = false;
            break;
        }
        else
        {
            valid_person = true;
        }
        }
    }
    else
    {
        valid_person = true;
    }
}

// callback_PCL: 1) If valid_person == false -> stops the car
//               2) Otherwise, it stores all the obstacle detections that are in front of the car
//                  and at a distance < 4 in the ordered map "valid_detections_map"
//               3) If the size of valid_detections_map == 0 -> No close obstancles -> Continue straight
//               4) Otherwise, steer left or right depending on where the closest obstacle is located
void ControlCar::callback_PCL(const vision_msgs::Detection3DArrayPtr &msg) {

    if (valid_person) {
        std::map<float, vision_msgs::Detection3D> valid_detections_map; // Ordered map

        int total_detections = (*msg).detections.size();

        // Loop through all obstacle detections
        for (int i = 0; i < total_detections; i++) {

        vision_msgs::Detection3D aux = (*msg).detections[i];

        float distance = sqrt(pow(aux.bbox.center.position.x, 2) + 
                                pow(aux.bbox.center.position.y, 2));

        // Store the "valid" obstacle detections in an ordered map
        if ((aux.bbox.center.position.x > 0) && (distance < 4.0)) {
            valid_detections_map[distance] = aux; 
        }
        }

        // Control alrgorithm
        if (valid_detections_map.size() == 0) {
        prius_msg.throttle = 1;
        prius_msg.steer = 0;
        prius_msg.brake = 0;
        } 
        else {
        std::map<float, vision_msgs::Detection3D>::iterator closest_detection_map = valid_detections_map.begin();

        float closest_distance = closest_detection_map->first;
        vision_msgs::Detection3D closest_detection = closest_detection_map->second;

        if (closest_detection.bbox.center.position.y > 0) {
            prius_msg.throttle = 1;
            prius_msg.steer = -1;
            prius_msg.brake = 0;
        } else {
            prius_msg.throttle = 1;
            prius_msg.steer = 1;
            prius_msg.brake = 0;
        }
        }
    } else {
        prius_msg.throttle = 0;
        prius_msg.steer = 0;
        prius_msg.brake = 1;
    }

    // Publish prius_msg to /prius topic
    pub.publish(prius_msg);
}

// Default Constructor
ControlCar::ControlCar() 
{
    // Publish control messages on /prius topic
    pub = nh_.advertise<prius_msgs::Control>("/prius", 1000);

    // Subscribe to OpenCV messages
    sub_openCV = nh_.subscribe("/opencv_solution_node/detections", 1000,
                                &ControlCar::callback_openCV, this);

    // Subscribe to PCL messages
    sub_PCL = nh_.subscribe("/pcl_solution_node/detections", 1000,
                            &ControlCar::callback_PCL, this);
}
