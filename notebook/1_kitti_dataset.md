# kitti dataset

website: https://www.cvlibs.net/datasets/kitti/



## 下载数据集

下载数据集需要先注册登陆。选择`raw data` 下载，种类选择`city`

后续都以**2011_09_26_drive_0005 (0.6 GB)**进行测试

下载内容有两个：

- unsynced+unrectified data 没有校正的数据
- calibration 校准数据

## 将数据集转化为ros可以读的rosbag

使用一个python库 https://github.com/tomas789/kitti2bag

安装为：**pip install kitti2bag**

还要注意一下目录结构。

- 2011_09_26_drive_0005_sync
  - 2011_09_26
    - 2011_09_26_drive_0005_sync
    - calib_cam_to_cam.txt 矫正文件放在这
    - calib_imu_to_velo.txt 矫正文件放在这
    - calib_velo_to_cam.txt 矫正文件放在这
  - kitti_2011_09_26_drive_0005_synced.bag 运行后生成的地方

在2011_09_26同级目录下使用命令 `kitti2bag -t 2011_09_26 -r 0005 raw_synced .` 可以转化数据

碰到报错解决报错就可以了。 `source /opt/ros/melodic/setup.zsh` `rosdep update`

成功后会生成`.bag`文件

使用 `rqt_bat kitti_2011_09_26_drive_0005_synced.bag` 可以可视化bag

![image-20230328232200632](https://raw.githubusercontent.com/gxt-kt/picgo_pictures/master/image-20230328232200632.png)

图片都可以可视化，点云不可以
