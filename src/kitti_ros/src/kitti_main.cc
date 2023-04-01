#include <string>
#include <vector>

#include "ros/ros.h"

// debugstream by gxt_kt
#include "kitti_ros/debugstream.hpp"

// pub to rviz
#include "kitti_ros/cloud_points.hpp"
#include "kitti_ros/gps.hpp"
#include "kitti_ros/images.hpp"
#include "kitti_ros/imu.hpp"
#include "kitti_ros/marker_car.hpp"
#include "kitti_ros/marker_line.hpp"
#include "kitti_ros/tracking.hpp"

// dataset directory
std::string dir =
    "/media/home/2011_09_26_drive_0005_sync/2011_09_26/"
    "2011_09_26_drive_0005_sync/";

std::string image_dir = dir + "image_02/data/";
std::string lidar_dir = dir + "velodyne_points/data/";
std::string imu_dir = dir + "oxts/data/";

std::string tracking_dir = dir + "training/label_02/0000.txt";

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "main_publish");
  ros::NodeHandle nh;
  gDebug << "main_publish";

  // image pub
  SendImage pub_image(nh, "camera02/image", 10);

  // cloudpoint pub
  CloudPoint pub_cloudpoint(nh, "cloudpoint", 50);

  // marker line pub
  SendMarkerLine pub_line(nh, "line_pub");

  // marker car pub
  SendMarkerCar pub_car(nh, "car_pub");

  // marker car pub
  SendImu pub_imu(nh, "imu_pub");

  // marker car pub
  SendGps pub_gps(nh, "gps_pub");

  // test tracking draw rectangle
  // cv::Mat img=cv::imread(image_dir+"0000000000.png");
  // TrackingDrawRect(img,tracking_dir,0);
  // cv::imshow("111",img);
  // cv::waitKey(0);
  // gDebug << "end";
  // return 0;

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
    // pub_image.Publish(image_dir + katti_num + ".png");  // Not use tracking
    pub_image.Publish(image_dir + katti_num + ".png", tracking_dir,
                      kitti_i);  // Use tracking

    // send lidar cloud_point
    pub_cloudpoint.Publish(lidar_dir + katti_num + ".bin");

    // send line marker
    pub_line.Publish();

    // send car marker
    pub_car.Publish();

    // send imu
    pub_imu.Publish(imu_dir + katti_num + ".txt");

    // send gps
    pub_gps.Publish(imu_dir + katti_num + ".txt");

    gDebug << "send kitti dataset i =" << kitti_i;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
