#ifndef IMAGES_HPP
#define IMAGES_HPP

#include <ros/ros.h>

#include <iostream>
#include <opencv2/highgui.hpp>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "kitti_ros/tracking.hpp"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

class SendImage {
 public:
  SendImage(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    image_transport::ImageTransport it(nh);
    pub = it.advertise(title, queue_size);
  };

  bool Publish(std::string image_path, std::string tracking_path = "",
               int img_frame = -1) {
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
      std::cerr << "error: image_path is error" << std::endl;
      return 1;
    }

    // draw rectangle use tracking data
    if (img_frame != -1) {
      TrackingDrawRect(image, tracking_path, img_frame);
    }

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pub.publish(msg);
    return 0;
  }

 private:
  ros::NodeHandle nh;
  std::string title;
  image_transport::Publisher pub;
};

#endif
