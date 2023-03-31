#ifndef MARKER_CAR_HPP
#define MARKER_CAR_HPP
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h> // tf转换


#include <string>

class SendMarkerCar {
 public:
  SendMarkerCar(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    pub = nh.advertise<visualization_msgs::Marker>(title, queue_size);
    InitialMarkerCar();
  };

  void InitialMarkerCar() {
    // 设置Marker消息的基本属性
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "car_marker";
    marker.id = -1;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://kitti_ros/meshes/car.stl";  // 这里的路径应该根据你的车模型的路径进行更改
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -1.73; // set position
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI); // roll,pitch,yaw
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }

  void Publish() { pub.publish(marker); }

 private:
  ros::NodeHandle nh;
  std::string title;
  ros::Publisher pub;
  visualization_msgs::Marker marker;
};

#endif
