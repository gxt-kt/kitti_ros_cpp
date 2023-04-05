# 在3dbox上显示出track_id

使用marker的`TEXT_VIEW_FACING`就可以显示文本了

直接在3dbox的坐标上显示出对应的track_id就好了



## 代码

为片段代码，实际上位置在3dbox后加入，并且将文本的marker加入到3dbox的marker_array中

```cpp
visualization_msgs::Marker marker_id;
marker_id.header.frame_id = "map";
marker_id.header.stamp = ros::Time::now();
marker_id.ns = "marker_id";
marker_id.id = id;
marker_id.lifetime = ros::Duration(0.1);  // 一秒十帧，所以框持续0.1s就消失
marker_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
marker_id.action = visualization_msgs::Marker::ADD;
marker_id.pose.position.x = (lidar_point(0, 4)+lidar_point(0, 5))/2;
marker_id.pose.position.y = (lidar_point(1, 4)+lidar_point(1, 5))/2;
marker_id.pose.position.z = (lidar_point(2, 4)+lidar_point(2, 5))/2;
marker_id.scale.z = 1.0;
marker_id.color.r = color.val[2] / 255;
marker_id.color.g = color.val[1] / 255;
marker_id.color.b = color.val[0] / 255;
marker_id.color.a = 1.0;
marker_id.text = id_string;

marker_array.markers.push_back(marker_id);
```



## 展示

![image-20230403234841320](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230403234841320.png)



