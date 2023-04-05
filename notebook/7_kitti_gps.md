# 发布gps信息

和imu一样，读取的文件在`oxts/data/`中，使用ifstream依次读取和写入就好了

在这里，其实可以避免重复读取，不过为了方便使用和演示，大部分代码就复用了imu的，没有进行删减



## 代码

```cpp
P
#define GPS_HPP
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <string>
#include <vector>

#include "kitti_ros/oxts_data_format.h"

class SendGps {
 public:
  SendGps(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    pub = nh.advertise<sensor_msgs::NavSatFix>(title, queue_size);
  };

  bool Publish(std::string kitti_imu_path) {
    std::ifstream imu_file(kitti_imu_path);

    if (!imu_file.is_open()) {
      std::cerr << "Unable to open file: " << kitti_imu_path << std::endl;
      return 1;
    }
    std::string line;
    std::getline(imu_file, line);
    std::istringstream iss(line);

    std::vector<double> gps_data(30);
    iss >> gps_data[0] >> gps_data[1] >> gps_data[2] >> gps_data[3] >>
        gps_data[4] >> gps_data[5] >> gps_data[6] >> gps_data[7] >>
        gps_data[8] >> gps_data[9] >> gps_data[10] >> gps_data[11] >>
        gps_data[12] >> gps_data[13] >> gps_data[14] >> gps_data[15] >>
        gps_data[16] >> gps_data[17] >> gps_data[18] >> gps_data[19] >>
        gps_data[20] >> gps_data[21] >> gps_data[22] >> gps_data[23] >>
        gps_data[24] >> gps_data[25] >> gps_data[26] >> gps_data[27] >>
        gps_data[28] >> gps_data[29];
    // std::cout << imu_data[roll] << imu_data[pitch] << imu_data[yaw]
    //           << std::endl;
    
    // 创建一个 NavSatFix 消息
    sensor_msgs::NavSatFix gps_msg;

    // 设置 GPS 数据
    gps_msg.latitude = gps_data[lat];
    gps_msg.longitude = gps_data[lon];
    gps_msg.altitude = gps_data[alt];
    gps_msg.header.frame_id = "map";
    gps_msg.header.stamp = ros::Time::now();
    pub.publish(gps_msg);
    return 0;
  }

 private:
  ros::NodeHandle nh;
  std::string title;
  ros::Publisher pub;
};

#endif
```



## 演示

由于rviz不能可视化gps数据

我们可以使用`rostopic`手动查看主题发布的消息

`rostopic list` 查看所有的话题

`rostopic info /gps_pub` 查看话题详情

![image-20230331222317542](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230331222317542.png)

`rostopic echo /gps_pub` 输出消息

![image-20230331222412164](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230331222412164.png)