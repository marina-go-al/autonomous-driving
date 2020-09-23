#include "../include/pcl_class.h"

using namespace std;

// planar_segmentation function which returns the new "cloud" object
// after the ground plane removal has been performed
inline pcl::PointCloud<pcl::PointXYZ>::Ptr DetectObstacle::planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object for the planar model and set all the
    // parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud = *cloud_f;

    return cloud;
}

// cluster_extraction function which detects the clusters after the planar_segmentation
// has been applied
inline std::vector<pcl::PointIndices> DetectObstacle::cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices){

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    return cluster_indices;
}

// callbackConversion: 1) Converts the input messages to the appropriate pcl::PointCloud<pcl::PointXYZ> type
//                     2) Performs the planar segmentation of the cloud
//                     3) Performs the cluster extraction of the cloud
//                     4) Converts the detections to the appropriate message type  vision_msgs::Detection3DArray
//                     5) Publishes this information on the /pcl_solution_node/detections topic
void DetectObstacle::callbackConversion(const sensor_msgs::PointCloud2Ptr &msg) {

    vision_msgs::Detection3DArray message_detections;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;

    // Message conversion
    pcl::fromROSMsg(*msg, *cloud);

    // Planar segmentation
    cloud = planar_segmentation(cloud);

    // Cluster extraction
    cluster_indices = cluster_extraction(cloud,cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it =
                cluster_indices.begin();
            it != cluster_indices.end(); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin();
            pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(cloud->points[*pit]); //*

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        // Auxiliary vision_msgs::Detection3D object to store each obstacle's
        // center position and size
        vision_msgs::Detection3D aux;
        aux.bbox.center.position.x = centroid[0];
        aux.bbox.center.position.y = centroid[1];
        aux.bbox.center.position.z = centroid[2];

        pcl::PointXYZ min, max;
        pcl::getMinMax3D(*cloud_cluster, min, max);
        aux.bbox.size.x = (max.x - min.x);
        aux.bbox.size.y = (max.y - min.y);
        aux.bbox.size.z = (max.z - min.z);
        aux.header = (*msg).header;

        // Array in which all the obstacle's information is stored
        message_detections.detections.push_back(aux);
    }
    // Publish the detections
    pub.publish(message_detections);
}

// Default Constructor
DetectObstacle::DetectObstacle() {

    // Publisher
    pub = nh_.advertise<vision_msgs::Detection3DArray>("/pcl_solution_node/detections", 1);

    //Subscriber
    sub = nh_.subscribe("/point_cloud", 1, &DetectObstacle::callbackConversion, this);
}
