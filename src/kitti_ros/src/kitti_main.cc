#include "ros/ros.h"
#include <string>
#include <vector>

// use cv_bridge to publish image
#include <opencv2/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

// debugstream by gxt_kt
#include "kitti_ros/debugstream.hpp"

// dataset directory
std::string dir =
    "/media/home/2011_09_26_drive_0005_sync/2011_09_26/"
    "2011_09_26_drive_0005_sync/image_02/data/";

int main(int argc, char *argv[]) {
  // 执行 ros 节点初始化
  ros::init(argc, argv, "main_publish");
  // 创建 ros 节点句柄(非必须)
  ros::NodeHandle nh;
  // 控制台输出 hello world
  // ROS_INFO("hello world!");
  gDebug << "hello world";
  std::string image_path = dir + "0000000001.png";
  std::vector<cv::String> fn;
  glob(dir + "*.png", fn, false);

  // cv::Mat im=cv::imread(fn[0]);
  // if (im.empty()) {
  //   gDebug << FATAL_ERROR << "Could not read the image: " << image_path;
  //   return 1;
  // }
  // imshow("image", im);
  // while ((cv::waitKey(0) & 0xFF) != 'q') {}

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera02/image", 10);
  ros::Rate loop_rate(10);
  while (nh.ok()) {
    {// send_image
      static size_t image_i{0};
      if (image_i >= fn.size()) image_i = 0;
      cv::Mat image = cv::imread(fn[image_i]);
      // cv::imshow("image",image);
      if (image.empty()) {
        gDebug << FATAL_ERROR << "error image is empty";
        return 1;  // only proceed if sucsessful
      }
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = "map";
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
      pub.publish(msg);
      gDebug << "send message" << image_i;
      ++image_i;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
