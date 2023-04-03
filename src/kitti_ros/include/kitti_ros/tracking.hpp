#ifndef TRACKING_HPP
#define TRACKING_HPP

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

#include "kitti_ros/tracking_format.h"

/*
#Values    Name      Description
----------------------------------------------------------------------------
   1    frame        Frame within the sequence where the object appearers
   1    track id     Unique tracking id of this object within this sequence
   1    type         Describes the type of object: 'Car', 'Van', 'Truck',
                     'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
                     'Misc' or 'DontCare'
   1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
                     truncated refers to the object leaving image boundaries.
                     Truncation 2 indicates an ignored object (in particular
                     in the beginning or end of a track) introduced by manual
                     labeling.
   1    occluded     Integer (0,1,2,3) indicating occlusion state:
                     0 = fully visible, 1 = partly occluded
                     2 = largely occluded, 3 = unknown
   1    alpha        Observation angle of object, ranging [-pi..pi]
   4    bbox         2D bounding box of object in the image (0-based index):
                     contains left, top, right, bottom pixel coordinates
   3    dimensions   3D object dimensions: height, width, length (in meters)
   3    location     3D object location x,y,z in camera coordinates (in meters)
   1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
   1    score        Only for results: Float, indicating confidence in
                     detection, needed for p/r curves, higher is better.
*/



inline bool TrackingDrawRect(cv::Mat& img, std::string tracking_path,
                             int img_frame = 0) {
  std::ifstream tracking_file(tracking_path);

  if (!tracking_file.is_open()) {
    std::cerr << "Unable to open file: " << tracking_path << std::endl;
    return 1;
  }
  std::string line;
  std::vector<std::string> tracking_data(17);
  while (std::getline(tracking_file, line)) {
    std::istringstream iss(line);
    if (!(iss >> tracking_data[0] >> tracking_data[1] >> tracking_data[2] >>
          tracking_data[3] >> tracking_data[4] >> tracking_data[5] >>
          tracking_data[6] >> tracking_data[7] >> tracking_data[8] >>
          tracking_data[9] >> tracking_data[10] >> tracking_data[11] >>
          tracking_data[12] >> tracking_data[13] >> tracking_data[14] >>
          tracking_data[15] >> tracking_data[16])) {
      break;
    }  // error
    if (std::stoi(tracking_data[frame]) == img_frame) {
      cv::Scalar color(255, 255, 255);
      auto find = tracking_color.find(tracking_data[type]);
      if (find != tracking_color.end()) {
        color = find->second;
      } else {
        color = tracking_color["OTHER"];
      }
      cv::rectangle(img,
                    cv::Rect(std::stoi(tracking_data[bbox_x1]),
                             std::stoi(tracking_data[bbox_y1]),
                             std::stoi(tracking_data[bbox_x2]) -
                                 std::stoi(tracking_data[bbox_x1]),
                             std::stoi(tracking_data[bbox_y2]) -
                                 std::stoi(tracking_data[bbox_y1])),
                    color);
    }

    if (std::stoi(tracking_data[frame]) > img_frame) {
      break;
    }
  }

  return 0;
}


inline bool TrackingDraw3dBox(cv::Mat& img, std::string tracking_path,
                              Eigen::Matrix<double, 3, 8>& matrix,
                              int img_frame = 0) {
  std::ifstream tracking_file(tracking_path);

  if (!tracking_file.is_open()) {
    std::cerr << "Unable to open file: " << tracking_path << std::endl;
    return 1;
  }
  std::string line;
  std::vector<std::string> tracking_data(17);
  while (std::getline(tracking_file, line)) {
    std::istringstream iss(line);
    if (!(iss >> tracking_data[0] >> tracking_data[1] >> tracking_data[2] >>
          tracking_data[3] >> tracking_data[4] >> tracking_data[5] >>
          tracking_data[6] >> tracking_data[7] >> tracking_data[8] >>
          tracking_data[9] >> tracking_data[10] >> tracking_data[11] >>
          tracking_data[12] >> tracking_data[13] >> tracking_data[14] >>
          tracking_data[15] >> tracking_data[16])) {
      break;
    }  // error
    if (std::stoi(tracking_data[frame]) == img_frame) {
      cv::Scalar color(255, 255, 255);
      auto find = tracking_color.find(tracking_data[type]);
      if (find != tracking_color.end()) {
        color = find->second;
      } else {
        color = tracking_color["OTHER"];
      }
      cv::rectangle(img,
                    cv::Rect(std::stoi(tracking_data[bbox_x1]),
                             std::stoi(tracking_data[bbox_y1]),
                             std::stoi(tracking_data[bbox_x2]) -
                                 std::stoi(tracking_data[bbox_x1]),
                             std::stoi(tracking_data[bbox_y2]) -
                                 std::stoi(tracking_data[bbox_y1])),
                    color);
    }

    if (std::stoi(tracking_data[frame]) > img_frame) {
      break;
    }
  }

  return 0;
}
#endif
