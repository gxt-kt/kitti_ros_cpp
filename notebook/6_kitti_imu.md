# rviz中显示imu

读取的文件在`oxts/data/`中，使用ifstream依次读取和写入就好了





## 代码

```cpp
#ifndef IMU_HPP
#define IMU_HPP
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>  // tf转换

#include <fstream>
#include <string>
#include <vector>

enum ImuDataFormat {
  lat = 0,
  lon,
  alt,
  roll,
  pitch,
  yaw,
  vn,
  ve,
  vf,
  vl,
  vu,
  ax,
  ay,
  az,
  af,
  al,
  au,
  wx,
  wy,
  wz,
  wf,
  wl,
  wu,
  pos_accuracy,
  vel_accuracy,
  navstat,
  numsats,
  posmode,
  velmode,
  orimode,
};

class SendImu {
 public:
  SendImu(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    pub = nh.advertise<sensor_msgs::Imu>(title, queue_size);
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

    std::vector<double> imu_data(30);
    iss >> imu_data[0] >> imu_data[1] >> imu_data[2] >> imu_data[3] >>
        imu_data[4] >> imu_data[5] >> imu_data[6] >> imu_data[7] >>
        imu_data[8] >> imu_data[9] >> imu_data[10] >> imu_data[11] >>
        imu_data[12] >> imu_data[13] >> imu_data[14] >> imu_data[15] >>
        imu_data[16] >> imu_data[17] >> imu_data[18] >> imu_data[19] >>
        imu_data[20] >> imu_data[21] >> imu_data[22] >> imu_data[23] >>
        imu_data[24] >> imu_data[25] >> imu_data[26] >> imu_data[27] >>
        imu_data[28] >> imu_data[29];
    // std::cout << imu_data[roll] << imu_data[pitch] << imu_data[yaw]
    //           << std::endl;

    sensor_msgs::Imu imu_msg;  // 创建一个IMU消息对象
    // 设置IMU消息中的数据
    imu_msg.linear_acceleration.x = imu_data[af];
    imu_msg.linear_acceleration.y = imu_data[al];
    imu_msg.linear_acceleration.z = imu_data[au];
    imu_msg.angular_velocity.x = imu_data[wf];
    imu_msg.angular_velocity.y = imu_data[wl];
    imu_msg.angular_velocity.z = imu_data[wu];
    tf::Quaternion q;
    q.setRPY(imu_data[roll], imu_data[pitch], imu_data[yaw]);  // roll,pitch,yaw
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    imu_msg.header.frame_id = "map";
    imu_msg.header.stamp = ros::Time::now();
    pub.publish(imu_msg);
    return 0;
  }

 private:
  ros::NodeHandle nh;
  std::string title;
  ros::Publisher pub;
};

#endif

```



## 展示

![image-20230331154354759](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230331154354759.png)