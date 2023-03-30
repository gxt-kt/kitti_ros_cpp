#ifndef MARKER_LINE_HPP
#define MARKER_LINE_HPP
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>

class SendMarkerLine {
 public:
  SendMarkerLine(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    pub = nh.advertise<visualization_msgs::Marker>(title, queue_size);
    InitialMarkerLine();
  };

  void InitialMarkerLine() {
    // Set our initial shape type to be a line strip
    uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

    // Set the marker type
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;

    // Set the marker action
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color of the marker
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;  // green
    marker.color.b = 0.0f;
    marker.color.a = 1.0;  // alpha

    // Set the lifetime of the marker
    marker.lifetime = ros::Duration();

    // Set the points of the line strip
    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = 10.0;
    p1.y = -10.0;
    p1.z = 0.0;
    p2.x = 0.0;
    p2.y = 0.0;
    p2.z = 0.0;
    p3.x = 10.0;
    p3.y = 10.0;
    p3.z = 0.0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
  }

  void Publish() { pub.publish(marker); }

 private:
  ros::NodeHandle nh;
  std::string title;
  ros::Publisher pub;
  visualization_msgs::Marker marker;
};

#endif
