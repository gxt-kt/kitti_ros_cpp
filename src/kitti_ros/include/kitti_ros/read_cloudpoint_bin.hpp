#ifndef READ_CLOUDPOINT_BIN_HPP
#define READ_CLOUDPOINT_BIN_HPP
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "kitti_ros/debugstream.hpp"

struct Point {
  float x;
  float y;
  float z;
  float intensity;
};

inline bool ConvertKittiBin2Point(std::string filename,
                                  std::vector<Point>& cloud_points) {
  std::ifstream input(filename, std::ios::binary);
  if (!input.good()) {
    std::cerr << "Could not read file: " << filename << std::endl;
    return 1;
  }

  Point point;

  while (input.read((char*)&point, sizeof(Point))) {
    cloud_points.push_back(point);
  }

  input.close();

  std::cout << "size=" << cloud_points.size();

  // 在这里对点云数据进行处理，例如使用PCL库将其转换为pcl::PointCloud<pcl::PointXYZI>类型

  return 0;
}


inline bool ConvertKittiBin2PclPointCloud(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  std::ifstream input(filename, std::ios::binary);
  if (!input.good()) {
    std::cerr << "Could not read file: " << filename << std::endl;
    return 1;
  }

  Point point;
  while (input.read((char*)&point, sizeof(Point))) {
    pcl::PointXYZI point_pcl;
    point_pcl.x = point.x;
    point_pcl.y = point.y;
    point_pcl.z = point.z;
    point_pcl.intensity = point_pcl.intensity;
    cloud->push_back(point_pcl);
  }

  input.close();

  return 0;
}
#endif
