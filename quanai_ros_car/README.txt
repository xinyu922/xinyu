编译：
1、创建catkin_ws工程文件
2、解压quanai_ros.tar.gz到src文件
3、编译工程文件
4、在无里程计的情况下，需要安装lidar_scan_matcher 包

运行：
1、启动雷达 
	roslaunch rplidar_ros rplidar_a3.launch
2、启动导航 
	roslaunch mrobot_navigation nav_demo.launch
3、lidar转odom
	roslaunch lidar_scan_matcher demo_gmapping.launch
4、python pub_cmd_vel.py
