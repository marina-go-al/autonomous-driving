#ifndef PCL_CLASS_H
#define PCL_CLASS_H

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>

class DetectObstacle {

private:
  // ROS Objects
  ros::NodeHandle nh_;

  // ROS Publishers
  ros::Publisher pub;

  // ROS Subscribers
  ros::Subscriber sub;

  // Functions encapsulation (planar segmentation, cluster extractiona and callback)
  inline pcl::PointCloud<pcl::PointXYZ>::Ptr planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  inline std::vector<pcl::PointIndices> cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices);
  void callbackConversion(const sensor_msgs::PointCloud2Ptr &msg);

public:
  // Default Constructor
  DetectObstacle();
};

#endif