#include "../include/opencv_class.h"

using namespace std;

// Default Constructor
DetectPerson::DetectPerson() {

// Publishers
pub_detections = nh_.advertise<vision_msgs::Detection2DArray>("/opencv_solution_node/detections", 1);
pub_visual = nh_.advertise<sensor_msgs::Image>("/opencv_solution_node/visual", 1);

// Subscriber
sub = nh_.subscribe("prius/front_camera/image_raw", 1, &DetectPerson::callbackConversion, this);
}


// callbackConversion: 1) Gets the input messages from the camera
//                     2) Converts them to a cv::Mat format (cv_ptr->image)
//                     3) Detects people with the OpenCV HOG person detector
//                     4) Keep the detection rectangules as a cv::Rect format
//                     5) Publish a vision_msgs::Detection2DArray message with all the
//                        detections on the corresponding topics
void DetectPerson::callbackConversion(const sensor_msgs::ImageConstPtr &msg) {

vision_msgs::Detection2DArray msg_detections;

// Initial conversion
cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

// Person detection
cv::HOGDescriptor hog;
hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
vector<cv::Rect> found;
hog.detectMultiScale(cv_ptr->image, found, 0, cv::Size(8, 8));

for (vector<cv::Rect>::iterator i = found.begin(); i != found.end(); ++i) {
    cv::Rect &r = *i;
    
    cv::rectangle(cv_ptr->image, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);

    // Save each detection's dimension and location
    vision_msgs::Detection2D aux;

    aux.bbox.center.x = r.x + r.width / 2;
    aux.bbox.center.y = r.y - r.height / 2;
    aux.bbox.size_x = r.width;
    aux.bbox.size_y = r.height;

    // Create a vector with all the detections
    msg_detections.detections.push_back(aux);
}

msg_detections.header = msg->header;

// Publish the messages to the corresponding topics
pub_visual.publish(cv_ptr->toImageMsg());
pub_detections.publish(msg_detections);
}