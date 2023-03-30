#ifndef CLOUD_POINTS_HPP
#define CLOUD_POINTS_HPP
#include <ros/ros.h>

#include <iostream>
#include <string>

// use pointcloud to send lidar data
#include <sensor_msgs/PointCloud.h>

#include "kitti_ros/read_cloudpoint_bin.hpp"

class CloudPoint {
 public:
  CloudPoint(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    pub = nh.advertise<sensor_msgs::PointCloud2>(title, queue_size);
  };

  bool Publish(std::string kitti_bin_path) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    ConvertKittiBin2PclPointCloud(kitti_bin_path, cloud);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header.frame_id = "map";
    ros_cloud.header.stamp = ros::Time::now();
    pub.publish(ros_cloud);
    return 0;
  }

 private:
  ros::NodeHandle nh;
  std::string title;
  ros::Publisher pub;
};
#endif
