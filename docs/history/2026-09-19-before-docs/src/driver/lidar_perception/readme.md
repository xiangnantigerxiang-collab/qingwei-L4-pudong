# 使用说明
1. cd {project_path/start2}  

2. 编译：  
catkin_make  

3. 之后每开启一个终端， 首先source环境变量
source devel/setup.bash


4. 启动感知：  
rosrun lidar_perception lidar_perception_scan_node  

5. 启动rviz
rosrun rviz rviz -d rviz.rviz

6. 部分编译  
  > catkin_make --pkg lidar_perception   
或者  
  > catkin_make -DCATKIN_WHITELIST_PACKAGES=lidar_perception