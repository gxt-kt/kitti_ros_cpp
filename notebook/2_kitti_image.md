# 发送图像数据

我们选择目录下的image_02文件夹下的图片，共计154张图片

使用cv_brige 作为桥梁发送图像到话题，cv_brige将opencv的图像转化为ros的图像消息，在这里我们只用了单向转化，当然双向转化也是可以的

cv_brige ： http://wiki.ros.org/cv_bridge

![image-20230330171330297](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230330171330297.png)



## 有几个小点可以留意一下

**读取文件**有很多种方式，读取图片可以使用opencv更方便的自动递归

> 比如使用了opencv中的glob去递归读取一个目录下的图像文件
>
> ```cpp
> glob(dir + "*.png", fn, false);
> ```
> 指定读取所有png文件到fn中
> 使用时就用 `cv::Mat img = fn[i]`就行
>
> ***
>
> **但是我们不用这个方法**，原因是后面还有点云什么的，没有这些更方便的方式
>
> 为了更灵活一点，使用手动读取的方式，指定文件名
>
> ```cpp
> // get current dataset sequence i
> static int kitti_i{0};
> kitti_i %= 154;
> std::stringstream ss;
> ss << std::setw(10) << std::setfill('0') << std::to_string(kitti_i);
> std::string katti_num = ss.str();
> kitti_i++;
> ```
>
> 这样可以**自动补全0来满足文件名的要求**



再将图像转化为消息的时候需要指定header


> ```cpp
>std_msgs::Header header;
> header.stamp = ros::Time::now();
> header.frame_id = "map";
> ```
> header的stamp（邮戳？）就是当前时间
> header的frame_id就是指定坐标系



## 代码

代码整体就比较简单了，看起来会有点杂乱，不过整体还是清晰的。

下面的代码暂时还是用的opencv的glob函数递归文件，后面的示例会改到手动

```cpp
#include "ros/ros.h"
#include <string>
#include <vector>

// use cv_bridge to publish image
#include <opencv2/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"

// debugstream by gxt_kt
#include "kitti_ros/debugstream.hpp"

// dataset directory
std::string dir =
    "/media/home/2011_09_26_drive_0005_sync/2011_09_26/"
    "2011_09_26_drive_0005_sync/image_02/data/";

int main(int argc, char *argv[]) {
  // 执行 ros 节点初始化
  ros::init(argc, argv, "main_publish");
  // 创建 ros 节点句柄(非必须)
  ros::NodeHandle nh;
  // 控制台输出 hello world
  // ROS_INFO("hello world!");
  gDebug << "hello world";
  std::string image_path = dir + "0000000001.png";
  std::vector<cv::String> fn;
  glob(dir + "*.png", fn, false);

  // cv::Mat im=cv::imread(fn[0]);
  // if (im.empty()) {
  //   gDebug << FATAL_ERROR << "Could not read the image: " << image_path;
  //   return 1;
  // }
  // imshow("image", im);
  // while ((cv::waitKey(0) & 0xFF) != 'q') {}

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera02/image", 10);
  ros::Rate loop_rate(10);
  while (nh.ok()) {
    {// send_image
      static size_t image_i{0};
      if (image_i >= fn.size()) image_i = 0;
      cv::Mat image = cv::imread(fn[image_i]);
      // cv::imshow("image",image);
      if (image.empty()) {
        gDebug << FATAL_ERROR << "error image is empty";
        return 1;  // only proceed if sucsessful
      }
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = "map";
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
      pub.publish(msg);
      gDebug << "send message" << image_i;
      ++image_i;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
```

