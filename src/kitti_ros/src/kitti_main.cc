#include <string>
#include <vector>

#include "ros/ros.h"


// debugstream by gxt_kt
#include "kitti_ros/debugstream.hpp"
#include "kitti_ros/marker_line.hpp"
#include "kitti_ros/cloud_points.hpp"
#include "kitti_ros/images.hpp"

// dataset directory
std::string dir =
    "/media/home/2011_09_26_drive_0005_sync/2011_09_26/"
    "2011_09_26_drive_0005_sync/";

std::string image_dir = dir + "image_02/data/";
std::string lidar_dir = dir + "velodyne_points/data/";

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "main_publish");
  ros::NodeHandle nh;
  gDebug << "main_publish";

  // image pub
  SendImage pub_image(nh, "camera02/image", 10);

  // cloudpoint pub
  CloudPoint pub_cloudpoint(nh, "cloudpoint", 50);

  // marker pub
  SendMarkerLine pub_line(nh, "line_pub");

  ros::Rate loop_rate(10);
  while (nh.ok()) {
    // get current dataset sequence i
    static int kitti_i{0};
    kitti_i %= 154;
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << std::to_string(kitti_i);
    std::string katti_num = ss.str();
    kitti_i++;

    // send_image
    pub_image.Publish(image_dir + katti_num + ".png");

    // send lidar cloud_point
    pub_cloudpoint.Publish(lidar_dir + katti_num + ".bin");

    // send line marker
    pub_line.Publish();

    gDebug << "send kitti dataset i =" << kitti_i;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
