#ifndef TRACK_SHOW_HPP
#define TRACK_SHOW_HPP
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>
// #incldue <cmath>

#include "kitti_ros/oxts_data_format.h"

struct TrackShowPoint {
  double x;
  double y;
};

class SendTrackShow {
 public:
  SendTrackShow(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    pub = nh.advertise<visualization_msgs::Marker>(title, queue_size);
    InitialMarkerTrackShow();
  };

  void InitialMarkerTrackShow() {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "track_show";
    marker.id = 3;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
  }

  bool Publish(std::string kitti_imu_path, int frame = -1) {
    std::ifstream imu_file(kitti_imu_path);

    if (!imu_file.is_open()) {
      std::cerr << "Unable to open file: " << kitti_imu_path << std::endl;
      return 1;
    }
    std::string line;
    std::getline(imu_file, line);
    std::istringstream iss(line);

    std::vector<double> imu_data(30);
    iss >> imu_data[0] >> imu_data[1] >> imu_data[2] >> imu_data[3] >>
        imu_data[4] >> imu_data[5] >> imu_data[6] >> imu_data[7] >>
        imu_data[8] >> imu_data[9] >> imu_data[10] >> imu_data[11] >>
        imu_data[12] >> imu_data[13] >> imu_data[14] >> imu_data[15] >>
        imu_data[16] >> imu_data[17] >> imu_data[18] >> imu_data[19] >>
        imu_data[20] >> imu_data[21] >> imu_data[22] >> imu_data[23] >>
        imu_data[24] >> imu_data[25] >> imu_data[26] >> imu_data[27] >>
        imu_data[28] >> imu_data[29];

    // 需要用到的数据
    const double track_yaw = imu_data[yaw];
    const double track_x = imu_data[vf];
    const double track_y = imu_data[vl];
    static double last_yaw = 0;

    if (frame == 0) {
      points.clear();
    }

    if (!points.empty()) {
      // 计算旋转角度差值为当前旋转角度
      double yaw_change = track_yaw - last_yaw;
      double displacement = std::sqrt(track_x * track_x + track_y * track_y);
      displacement *= 0.1;  // 乘0.1是因为这个速度移动了0.1s，一秒十帧
      for (auto& point : points) {
        // 这里乘的就是旋转矩阵的逆
        double x_ = point.x * std::cos(yaw_change) +
                    point.y * std::sin(yaw_change) - displacement;
        double y_ =
            -point.x * std::sin(yaw_change) + point.y * std::cos(yaw_change);
        point.x = x_;
        point.y = y_;
      }
    }
    last_yaw = track_yaw;
    points.emplace_back(TrackShowPoint{0, 0});  // 加入当前坐标点

    // if (points.size() >= 154) {
    //   std::cout << "points size" << points.size() << std::endl;
    //   for (auto& point : points) {
    //     std::cout << "points" << point.x << " " << point.y << std::endl;
    //   }
    // }

    geometry_msgs::Point p;
    for (const auto& point : points) {
      p.x = point.x;
      p.y = point.y;
      p.z = 0;
      marker.points.push_back(p);
    }
    pub.publish(marker);
    marker.points.clear();
    return 0;
  }

 private:
  std::vector<TrackShowPoint> points;
  ros::NodeHandle nh;
  std::string title;
  ros::Publisher pub;
  visualization_msgs::Marker marker;
};

#endif
