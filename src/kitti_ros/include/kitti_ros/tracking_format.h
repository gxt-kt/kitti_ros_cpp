#ifndef TRACKINGFORMAT_H
#define TRACKINGFORMAT_H

#include <unordered_map>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

enum TrackingFormat {
  frame = 0,
  track_id,
  type,
  truncated,
  occluded,
  alpha,
  bbox_x1,
  bbox_y1,
  bbox_x2,
  bbox_y2,
  h_,
  w_,
  l_,
  x_,
  y_,
  z_,
  rotation_y,
  score,
};

static std::unordered_map<std::string, cv::Scalar_<double>> tracking_color{
    {"Car", cv::Scalar(255, 0, 0)},
    {"Van", cv::Scalar(0, 255, 0)},
    {"Truck", cv::Scalar(0, 0, 255)},
    {"Pedestrain", cv::Scalar(255, 255, 0)},
    {"Person_sitting", cv::Scalar(255, 0, 255)},
    {"Cyclist", cv::Scalar(0, 255, 255)},
    {"OTHER", cv::Scalar(255, 255, 255)},
};

#endif 
