# 使用tracking在图像中画2D框



## 下载tracking数据

![image-20230401225224105](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230401225224105.png)

下载后对应的training文件夹下会有`label_02`文件夹，里面对应的`0000.txt`就是我们对应视频的需要的标注文件。

>  文件内容含义可以参考 https://github.com/pratikac/kitti/blob/eba7ba0f36917f72055060e9e59f344b72456cb9/readme.tracking.txt这里
>
> ```
> #Values    Name      Description
> ----------------------------------------------------------------------------
>    1    frame        Frame within the sequence where the object appearers
>    1    track id     Unique tracking id of this object within this sequence
>    1    type         Describes the type of object: 'Car', 'Van', 'Truck',
>                      'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
>                      'Misc' or 'DontCare'
>    1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
>                      truncated refers to the object leaving image boundaries.
> 		     Truncation 2 indicates an ignored object (in particular
> 		     in the beginning or end of a track) introduced by manual
> 		     labeling.
>    1    occluded     Integer (0,1,2,3) indicating occlusion state:
>                      0 = fully visible, 1 = partly occluded
>                      2 = largely occluded, 3 = unknown
>    1    alpha        Observation angle of object, ranging [-pi..pi]
>    4    bbox         2D bounding box of object in the image (0-based index):
>                      contains left, top, right, bottom pixel coordinates
>    3    dimensions   3D object dimensions: height, width, length (in meters)
>    3    location     3D object location x,y,z in camera coordinates (in meters)
>    1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
>    1    score        Only for results: Float, indicating confidence in
>                      detection, needed for p/r curves, higher is better.
> ```



## 代码

说一下函数`TrackingDrawRect（cv::Mat& img, std::string tracking_path,
                             int img_frame = 0）`几个小点吧

- 第一个参数为图像，第二个为tracking文件路径，第三个当前图像是第几帧，在这个项目中肯定数值在`0～153`

- 为了让不同类型画出不同颜色的框，用了一个`unordered_map`来匹配配型和颜色，只特定了集中，其余都用白色框
- fstream读文件时采用一行一行读取，根据`img_frame`进行筛选，只有等于当前帧才把框放进去，如果大于当前帧则直接停止（因为文件中的帧是按从小到大来的，如果大了说明框肯定已经画完了）



```cpp
#ifnder TRACKING_HPP
#define TRACKING_HPP

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <string>
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
  dimensions,
  location,
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

#endif
```



我们只需要让图像发布前执行上面那个函数图像上画好框就可以了。

## 展示

![image-20230402005422092](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230402005422092.png)

