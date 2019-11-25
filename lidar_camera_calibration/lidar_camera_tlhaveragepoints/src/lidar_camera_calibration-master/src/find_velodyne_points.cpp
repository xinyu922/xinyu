#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "lidar_camera_calibration/Corners.h"
#include "lidar_camera_calibration/PreprocessUtils.h"
#include "lidar_camera_calibration/Find_RT.h"

#include "lidar_camera_calibration/marker_6dof.h"

using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;


string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;


Mat projection_matrix;

pcl::PointCloud<myPointXYZRID> point_cloud;


Eigen::Quaterniond qlidarToCamera;     //姿态变换的四元数
Eigen::Matrix3d lidarToCamera;          //姿态变换的旋转矩阵


void callback_noCam(const sensor_msgs::PointCloud2ConstPtr& msg_pc,
					const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt)
					{
    ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
    ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());

    // Loading Velodyne point cloud_sub
    fromROSMsg(*msg_pc, point_cloud);      //点云信息从ROS形式转成PCL形式，格式转换，点云用POINT——CLOUD量量替替

  point_cloud = transform(point_cloud, 0, 0, 0, config.initialRot[0], config.initialRot[1],config.initialRot[2]);   //，点云初始方位角转换,轴变换了

    //Rotation matrix to transform lidar point cloud to camera's frame

    qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())      //处理初始化旋转
                     * Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());

    lidarToCamera = qlidarToCamera.matrix();           //初始化旋转的记录，之后ICP时要补偿

    std::cout << "\n\nInitial Rot" << lidarToCamera << "\n";    //这里输出出来不知道干嘛的，感觉没用呀



////////////////////	point_cloud = intensityByRangeDiff(point_cloud, config);     //这个函数很重要，在PreprocessUtils.h里定义，用于过滤掉一些没用的雷达点云！！！！！！！！！



//    pcl::PointCloud<myPointXYZRID> intensityByRangeDiff(pcl::PointCloud<myPointXYZRID> point_cloud, config_settings config)
//    {
//
//        std::vector<std::vector<myPointXYZRID*>> rings(16);      //16线的意思，这是一个自定义的16线激光雷达存储类
//
//        for(pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin() ; pt < point_cloud.points.end(); pt++){
//            pt->range = (pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);         //算出一个激光雷达点云距离的平方，这里是在构造一个rings,但是后面没用到rings
//            rings[pt->ring].push_back(&(*pt));
//        }
//
//        for(std::vector<std::vector<myPointXYZRID*>>::iterator ring = rings.begin(); ring < rings.end(); ring++)    //这一段是干啥的，感觉对过滤没有任何影响？
//        {
//            myPointXYZRID* prev, *succ;
//            if (ring->empty())
//            {
//                continue;
//            }
//            float last_intensity = (*ring->begin())->intensity;
//            float new_intensity;
//            (*ring->begin())->intensity = 0;
//            (*(ring->end() - 1))->intensity = 0;
//            for (std::vector<myPointXYZRID*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++)
//            {
//                prev = *(pt - 1);
//                succ = *(pt + 1);
//
//
//                (*pt)->intensity = MAX( MAX( prev->range-(*pt)->range, succ->range-(*pt)->range), 0) * 10;    //MAX函数，返回比较大的
//            }
//        }
//        point_cloud = normalizeIntensity(point_cloud, 0.0, 1.0);      //强度归一化到0-1
//
//        pcl::PointCloud<myPointXYZRID> filtered;
//
//        for(pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin() ; pt < point_cloud.points.end(); pt++)       //用距离以及强度过滤
//        {
//            if(pt->intensity  >  config.intensity_thresh)             //强度过滤，远的强度肯定弱
//            {
//                if(pt->x >= config.xyz_[0].first && pt->x <= config.xyz_[0].second && pt->y >= config.xyz_[1].first && pt->y <= config.xyz_[1].second && pt->z >= config.xyz_[2].first && pt->z <= config.xyz_[2].second)
//                {                   //距离过滤
//                    filtered.push_back(*pt);
//                }
//            }
//        }
////config文件里有这个距离过滤参数设置
////    x- and x+, y- and y+, z- and z+ are used to remove unwanted points in the cloud and are specfied in meters. The filtred point cloud makes it easier to mark the board edges. The filtered pointcloud contains all points
////    (x, y, z) such that,
////    x in [x-, x+]
////    y in [y-, y+]
////    z in [z-, z+]
////    The cloud_intensity_threshold is used to filter points that have intensity lower than a specified value.
////    The default value at which it works well is 0.05. However, while marking, if there seem to be missing/less points on the cardboard edges,
////    tweaking this value will might help.
//
//
//
//
//
//        //pcl::io::savePCDFileASCII ("/home/vishnu/PCDs/filtered.pcd", *(toPointsXYZ(filtered)));
//        return filtered;
//    }



    // x := x, y := -z, z := y

    //pcl::io::savePCDFileASCII ("/home/vishnu/PCDs/msg_point_cloud.pcd", pc);
    pcl::PointCloud<myPointXYZRID> filtered;

    for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin();pt < point_cloud.points.end(); pt++)        //自己写的强度及距离过滤
    {
        if (pt->intensity < 50 && pt->intensity > 3) 
{
            if (pt->y >= -1 && pt->y <= 1.3 && pt->x >= -0.4 && pt->x <= 0.4&& pt->z >= 0.5 && pt->z <= 1.5)     //y:up,down x:left,right z:front,background
 {                   //距离过滤
//                    pcl::PointXYZI point;
//                    point.x=pt->x;
//                    point.y=pt->y;
//                    point.z=pt->z;
//                    point.intensity=pt->intensity;
//                    cout<<point.intensity<<" ";
                filtered.push_back(*pt);
            }
        }
    }
        point_cloud = filtered;


        cv::Mat temp_mat(config.s, CV_8UC3);     //应该是用来存图像的 CONFIG.S是图像尺寸,3通道
        pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));     //雷达点云数据   函数目的是是将云云输据转换成XYZ格式
  //  pcl::PointCloud<pcl::PointXYZ> *new_cloud = new pcl::PointCloud<pcl::PointXYZ>();
  //  for (pcl::PointCloud<PointXYZI>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
 //   {
 //       new_cloud->push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
 //   }
 //   pcl::PointCloud<pcl::PointXYZ> retval=*new_cloud;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//    pcl::PointCloud<pcl::PointXYZ>* toPointsXYZ(pcl::PointCloud<myPointXYZRID> point_cloud)
        //   {
        //       pcl::PointCloud<pcl::PointXYZ> *new_cloud = new pcl::PointCloud<pcl::PointXYZ>();
        //       for (pcl::PointCloud<myPointXYZRID>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
//        {
//            new_cloud->push_back(pcl::PointXYZ(pt->x, pt->y, pt->z));
        //       }
        //      return new_cloud;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////





        std::vector<float> marker_info;     //现在处理MARKER信息传过来的数据

        for (std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it) {
            marker_info.push_back(*it);          //将MARKER中的据据一一存到向量MARKER——INFO中并打印出来
            std::cout << *it << " ";
        }
        std::cout << "\n";

        getCorners(temp_mat, retval, config.P, config.num_of_markers,
                   config.MAX_ITERS);        //以上是数据接收阶段，现在进入CORNERS正题，传进去的参数为个个点云，MARKER参数，有有个个图像矩阵变量，该步骤最后能得到雷达坐标系下标定板四角点点云
        find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS,
                            lidarToCamera);          //里面就是icp!!!!  FIND——RT。H  //lidarToCamera是初始化旋转的记录，ICP时要补偿,点的信息在marker_info里
        //ros::shutdown();

}
void callback(const sensor_msgs::CameraInfoConstPtr& msg_info,
			  const sensor_msgs::PointCloud2ConstPtr& msg_pc,
			  const lidar_camera_calibration::marker_6dof::ConstPtr& msg_rt)
    {

	ROS_INFO_STREAM("Camera info received at " << msg_info->header.stamp.toSec());
	ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
	ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());

	float p[12];
	float *pp = p;
	for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin(); i != msg_info->P.end(); i++)
	{
	*pp = (float)(*i);
	pp++;
	}
	cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);



	// Loading Velodyne point cloud_sub
	fromROSMsg(*msg_pc, point_cloud);

	point_cloud = transform(point_cloud, 0, 0, 0, config.initialRot[0], config.initialRot[1], config.initialRot[2]);

	//Rotation matrix to transform lidar point cloud to camera's frame

	qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())
		*Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
		*Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());

	lidarToCamera = qlidarToCamera.matrix();

	point_cloud = intensityByRangeDiff(point_cloud, config);
	// x := x, y := -z, z := y

	//pcl::io::savePCDFileASCII ("/home/vishnu/PCDs/msg_point_cloud.pcd", pc);  


	cv::Mat temp_mat(config.s, CV_8UC3);
	pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

	std::vector<float> marker_info;

	for(std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it)
	{
		marker_info.push_back(*it);
		std::cout << *it << " ";
	}
	std::cout << "\n";

	getCorners(temp_mat, retval, projection_matrix, config.num_of_markers, config.MAX_ITERS);
	find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
	//ros::shutdown();
}


//image_width image_height
//x- x+
//y- y+
//z- z+
//cloud_intensity_threshold
//        number_of_markers
//use_camera_info_topic?
//fx 0 cx 0
//0 fy cy 0
//0 0 1 0
//
//MAX_ITERS
//
// initial_rot_x initial_rot_y initial_rot_z












int main(int argc, char** argv)
{
	readConfig();
	ros::init(argc, argv, "find_transform");

	ros::NodeHandle n;

	if(config.useCameraInfo)
	{
		ROS_INFO_STREAM("Reading CameraInfo from topic");
		n.getParam("/lidar_camera_calibration/camera_info_topic", CAMERA_INFO_TOPIC);   //CAMERA_INFO_TOPIC是info的topic名字
		n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);

		message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(n, CAMERA_INFO_TOPIC, 1);      //得到info   总共三个topic，从aruco那里接收的只有lidar_camera_calibration_rt
		message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
		message_filters::Subscriber<lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);

		std::cout << "done1\n";

		typedef sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, lidar_camera_calibration::marker_6dof> MySyncPolicy;
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), info_sub, cloud_sub, rt_sub);
		sync.registerCallback(boost::bind(&callback, _1, _2, _3));

		ros::spin();
	}
	else        //no camera info
	{
		ROS_INFO_STREAM("Reading CameraInfo from configuration file");   //输出Reading CameraInfo from configuration file
  		n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);    //velodyne_points

		message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);  //信息滤器器，订阅者过滤器是对ROS订阅的封装，为其他过滤器提供源代码，订阅者过滤器无法将另一个过滤器的输出作为其输入，而是使用ROS主题作为其输入。
		message_filters::Subscriber<lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);

		typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, lidar_camera_calibration::marker_6dof> MySyncPolicy;//时间同步器：TimeSynchronizer过滤器通过包含在其头中的时间戳来同步输入通道，并以单个回调的形式输出它们需要相同数量的通道。 C ++实现可以同步最多9个通道。
		Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, rt_sub);
		sync.registerCallback(boost::bind(&callback_noCam, _1, _2));//步后后用用函数数&callback_noCam，_1,_2就是cloud_sub, rt_sub，即为回函数数的参参参数

		ros::spin();
	}

	return EXIT_SUCCESS;
}
