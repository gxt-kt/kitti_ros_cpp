# rviz中画小车模型

同样使用ros中的Marker包画出模型

可以支持stl，dae，obj等模型文件，但不是所有的都支持，stl也需要使用ASCII保存。如果不支持换一个模型就好恶劣

## 代码

- 注意每个marker的`frameid`一定要不一样，否则只会一个frameid只会显示一个，frameid可以为负数

- 注意路径`package://kitti_ros/meshes/car.stl`

  格式为：`package://功能包/路径`，我这里是放到了meshes文件夹下了

  **记得打开rviz前先source一下**，否则会提示找不到文件的

- 小车的模型位置和方向可能需要调整

  位置z轴为-1.73m，和kitti的数据集的位置保持一致

  旋转方向和模型有关，我这里需要z轴旋转180度，代码中使用`tf`将欧拉角转换为四元数

```cpp
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

```

## 展示

![image-20230331145014993](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230331145014993.png)

