
##查看相机的坐标
roslaunch realsense2_camera rs_d435_camera_with_model.launch


##订阅三维点云文件
获得三维点云rgbd信息，启动如下launch文件
` roslaunch realsense2_camera rs_rgbd.launch `
然后订阅 /camera/depth_registered/points话题的信息，拿来后处理就行。


2.相机内外参的标定在哪里修改

## rgbd深度图的坐标系是那个，与相机坐标系的关系。
rostopic echo /camera/depth_registered/points | grep frame_id
对齐之后的rgbd深度图的坐标系是以rgb的坐标系frame_id: "camera_color_optical_frame"为基准的

如下几何关系为与相机坐标系的关系
z'=4.2mm
Z轴偏移25.05-4.2 = 20.84mm

X轴32.5mm

3.rviz中红色为X轴，绿色为Y轴，蓝色为Z轴。


ROS功能包已经发布了点云，能否拿来直接用？
可以拿来用

怎么确定获取点云图像的长和宽

##使用plc_tool查看pcd文件
`   sudo apt install pcl-tools   `
`   pcl_viewer xxx.pcd    `

### 录制数据
`rosbag record -a `

##播放录制的数据

` rosbag play -r 1 realsense.bag ` # (这里在你录制的包的路径下运行)  1倍速回放
