# 画出小车轨迹

## 公式计算

小车的坐标始终为（0，0）

步骤：

1. 使用一个vector或者queue，存储过去点的信息

2. 每经过一帧就将过去存储的所有点进行一次更新

3. 这样就是vector的第一个数据为历史最远点的位置

   vector的最后一个数据就是当前的坐标（0，0）



坐标变换关键点：

- 坐标系变化如下图左边显示
- 计算的旋转矩阵加平移如下图右边的框显示
  - 注意旋转矩阵是以当前帧计算上一帧的，所以是原本二维旋转矩阵的逆
  - 为什么需要在x方向减去d，d是距离：因为距离是按照当前帧x轴负向计算的，坐标就是之间减去d



![image-20230405000823072](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230405000823072.png)



## 代码

### 计算和存储点

注意计算距离时需要乘以0.1，是因为移动了0.1s，一秒十帧，计算后就是距离

```cpp
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
```



### 画曲线

使用marker的`LINE_STRIP`

```cpp
// 初始化
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


// 使用
geometry_msgs::Point p;
for (const auto& point : points) {
  p.x = point.x;
  p.y = point.y;
  p.z = 0;
  marker.points.push_back(p);
}
pub.publish(marker);
marker.points.clear();

```



## 展示



![image-20230405095929615](https://github.com/gxt-kt/picgo_pictures/blob/master/image-20230405095929615.png)





## 额外的

在这里我们使用的vector保存了历史的所有轨迹

也可以使用queue固定队列长度，保存固定的历史点数