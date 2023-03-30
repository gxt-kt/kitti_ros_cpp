#include <string>
#include <vector>

#include "ros/ros.h"

// use cv_bridge to publish image
#include <opencv2/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

// use pointcloud to send lidar data
#include <sensor_msgs/PointCloud.h>

// debugstream by gxt_kt
#include "kitti_ros/debugstream.hpp"
#include "kitti_ros/read_cloudpoint_bin.hpp"

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
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("camera02/image", 10);

  // cloudpoint pub
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cloudpoint", 50);


  ros::Rate loop_rate(10);
  while (nh.ok()) {

    // get current dataset sequence i
    static int kitti_i{0};
    kitti_i %= 154;
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << std::to_string(kitti_i);
    std::string katti_num = ss.str();
    kitti_i++;

    {  // send_image
      cv::Mat image = cv::imread(image_dir + katti_num + ".png");
      // cv::imshow("image",image);
      if (image.empty()) {
        gDebug << FATAL_ERROR << "error image is empty";
        return 1;
      }
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = "map";
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
      image_pub.publish(msg);
    }

    {  // send lidar cloud_point
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZI>);
      ConvertKittiBin2PclPointCloud(lidar_dir + katti_num+".bin", cloud);
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*cloud, ros_cloud);
      ros_cloud.header.frame_id = "map";
      ros_cloud.header.stamp = ros::Time::now();
      cloud_pub.publish(ros_cloud);
    }

    gDebug << "send kitti dataset i =" << kitti_i;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
