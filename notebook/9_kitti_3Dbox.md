# 画出3dbox

## 先根据数据弄出框的位置来

3dbox 总共需要7个数据，分别为3dbox的h,w,l, 固定点的xyz和一个偏角（从天上往地下看旋转）

xyz的坐标系定义按照camera的定义的，深度方向是z，所以旋转时的偏角yaw是按照y轴来旋转的

整体的图可以看这个草图

![image-20230402231012906](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230402231012906.png)



### 代码

由于3dbox有三个顶点，我们放到一起算，定义3x8的矩阵，然后偏角单独计算一个旋转矩阵，乘上之前的矩阵就是旋转后的位置，最后再加上xyz就是最终的点了

这个点是在camera坐标系下的点

```cpp
inline Eigen::Matrix<double, 3, 8> Compute3Dbox(double h, double w, double l,
                                                double x, double y, double z,
                                                double rotation) {
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rotation, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  Eigen::Matrix<double, 3, 8> points;
  points.setZero();
  points << l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2, 0, 0, 0,
      0, -h, -h, -h, -h, w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2,
      w / 2;

  Eigen::Matrix<double, 3, 8> rotate_points = rotation_matrix * points;

  rotate_points.row(0) += x * Eigen::VectorXd::Ones(8);  // 加上原坐标xyz
  rotate_points.row(1) += y * Eigen::VectorXd::Ones(8);
  rotate_points.row(2) += z * Eigen::VectorXd::Ones(8);

  return rotate_points;
}

```


## 再把camera坐标系的框转到lidar坐标系下

### 代码

这里的`R t`是从文件`calib_velo_to_cam.txt`拿到了下的

实际求解是对4*4的矩阵T取逆就是从相机到点云的转移矩阵了

```cpp
// 将camera的点转到lidar点云坐标系下
inline Eigen::Matrix<double, 3, 8> CalibrationPoint(
    Eigen::Matrix<double, 3, 8> points) {
  Eigen::Matrix3d R;
  R << 7.533745e-03, -9.999714e-01, -6.166020e-04, 1.480249e-02, 7.280733e-04,
      -9.998902e-01, 9.998621e-01, 7.523790e-03, 1.480755e-02;
  Eigen::Vector3d t;
  t << -4.069766e-03, -7.631618e-02, -2.717806e-01;
  Eigen::Isometry3d T(R);
  T.pretranslate(t);

  Eigen::Vector3d t2(points(0, 0), points(1, 0), points(2, 0));
  Eigen::Matrix<double, 3, 8> rotate_points = T.inverse() * points;

  return rotate_points;
}
```



## 可视化显示

这一部分用到了marker的`LINE_LIST`实现画一个长方体

并用到了markerarray实现了将一帧中所有的长方体放入集合当中，这样只需要发送一次就好了

其余小点：

- 在tracking文件中，会有一种类型为DontCare，就直接忽略掉，不画就好了
- 这一次，我们用到了之前提到过的持续时间，由于一秒十帧，持续时间就设置为0.1s，这样每个框在0.1s后就消失了，下一次重新画

```cpp
class SendMarker3dBox {
 public:
  SendMarker3dBox(ros::NodeHandle nh_, std::string title_, int queue_size = 10)
      : nh(nh_), title(title_) {
    pub = nh.advertise<visualization_msgs::MarkerArray>(title, queue_size);
  };

  bool TrackingDraw3dBox(std::string tracking_path, int img_frame = 0) {
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
        if(tracking_data[type]=="DontCare") {
          continue;
        }
        cv::Scalar color(255, 255, 255);
        auto find = tracking_color.find(tracking_data[type]);
        if (find != tracking_color.end()) {
          color = find->second;
        } else {
          color = tracking_color["OTHER"];
        }
        // 根据对应帧执行操作
        AddMarker3dBox(
            std::stod(tracking_data[h_]), std::stod(tracking_data[w_]),
            std::stod(tracking_data[l_]), std::stod(tracking_data[x_]),
            std::stod(tracking_data[y_]), std::stod(tracking_data[z_]),
            std::stod(tracking_data[rotation_y]),
            std::stoi(tracking_data[track_id]), color);
      }

      if (std::stoi(tracking_data[frame]) > img_frame) {
        break;
      }
    }

    return 0;
  }
  void AddMarker3dBox(double h, double w, double l, double x, double y,
                      double z, double rotation, int id,
                      cv::Scalar_<double> color) {
    // Create the marker message
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = color.val[2]/255;
    marker.color.g = color.val[1]/255;
    marker.color.b = color.val[0]/255;
    marker.color.a = 1.0;

    // Set the marker type
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the scale of the marker
    marker.scale.x = 0.1;

    // Set the lifetime of the marker
    marker.lifetime = ros::Duration(0.1);  // 一秒十帧，所以框持续0.1s就消失

    Eigen::Matrix<double, 3, 8> matrix =
        Compute3Dbox(h, w, l, x, y, z, rotation);
    Eigen::Matrix<double, 3, 8> lidar_point = CalibrationPoint(matrix);

    std::vector<geometry_msgs::Point> p(8);  // 定义8个点
    for (int i{0}; i < p.size(); i++) {
      p.at(i).x = lidar_point(0, i);
      p.at(i).y = lidar_point(1, i);
      p.at(i).z = lidar_point(2, i);
    }

    marker.points.push_back(p.at(0));
    marker.points.push_back(p.at(1));
    marker.points.push_back(p.at(1));
    marker.points.push_back(p.at(2));
    marker.points.push_back(p.at(2));
    marker.points.push_back(p.at(3));
    marker.points.push_back(p.at(3));
    marker.points.push_back(p.at(0));

    marker.points.push_back(p.at(4));
    marker.points.push_back(p.at(5));
    marker.points.push_back(p.at(5));
    marker.points.push_back(p.at(6));
    marker.points.push_back(p.at(6));
    marker.points.push_back(p.at(7));
    marker.points.push_back(p.at(7));
    marker.points.push_back(p.at(4));

    marker.points.push_back(p.at(4));
    marker.points.push_back(p.at(0));
    marker.points.push_back(p.at(5));
    marker.points.push_back(p.at(1));
    marker.points.push_back(p.at(6));
    marker.points.push_back(p.at(2));
    marker.points.push_back(p.at(7));
    marker.points.push_back(p.at(3));

    marker_array.markers.push_back(marker);
  }

  void Publish(std::string tracking_path = "", int img_frame = -1) {
    TrackingDraw3dBox(tracking_path, img_frame);
    pub.publish(marker_array);
    marker_array.markers.clear();
  }

 private:
  ros::NodeHandle nh;
  std::string title;
  ros::Publisher pub;
  visualization_msgs::MarkerArray marker_array;
};

```



## 展示

![image-20230403234214459](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230403234214459.png)
