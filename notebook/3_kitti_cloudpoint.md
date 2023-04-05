# 发送点云数据

发布点云使用的是ros的pointcloud2d



kitti的点云文件是用bin文件存储的，一个bin文件大概有1w+组点

每一行一个点，共四个数，依次是[x,y,z,intensity] 也就是3d坐标加上反射度

每个数都是float32型，4个字节，每行就是4*4=16个字节

**读取时把每行四个字节由二进制转换成flaot，依次读入到[x,y,z,intensity] 就好了**

下面来看代码，下面首先定义一个点的结构体

```cpp
struct Point {
  float x;
  float y;
  float z;
  float intensity;
};
```

然后是读取函数，输入是文件名和pcl的point存储的vector

```cpp
inline 
bool ConvertKittiBin2PclPointCloud(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    
  std::ifstream input(filename, std::ios::binary);
  if (!input.good()) {
    std::cerr << "Could not read file: " << filename << std::endl;
    return 1;
  }

  Point point;
  while (input.read((char*)&point, sizeof(Point))) {
    pcl::PointXYZI point_pcl;
    point_pcl.x = point.x;
    point_pcl.y = point.y;
    point_pcl.z = point.z;
    point_pcl.intensity = point_pcl.intensity;
    cloud->push_back(point_pcl);
  }

  input.close();

  return 0;
}
```



## 代码

加上小节的图像一起的代码是 注意这里已经改到了手动读取文件

```cpp
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

```

## 结果展示

![image-20230330232506188](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230330232506188.png)

在rviz中添加`image`和`pointcloud2`，就可以查看到可视化结果了

其中为了方便对应点云和图像，我用绿色和青色框圈出了骑自行车的人和汽车