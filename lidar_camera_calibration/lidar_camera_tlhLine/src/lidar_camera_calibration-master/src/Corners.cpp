#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <map>
#include <fstream>
#include <cmath>

#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h>


#include "lidar_camera_calibration/Utils.h"
int saveflag=1;
int iteration_count = 0;
std::vector< std::vector<cv::Point> > stored_corners;
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints4(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints5(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints6(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints7(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints8(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr savepoints9(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds4(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds5(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds6(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds7(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds8(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds9(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds10(new pcl::PointCloud<pcl::PointXYZ>);



//图像，点云，相机内参，标志数量，迭代次数
void getCorners(cv::Mat img, pcl::PointCloud<pcl::PointXYZ> scan, cv::Mat P, int num_of_markers, int MAX_ITERS) {

    ROS_INFO_STREAM("iteration number: " << iteration_count << "\n");







//////////这里就是构建那个选点的黑色图、、、
    /*Masking happens here */
    cv::Mat edge_mask = cv::Mat::zeros(img.size(), CV_8UC1); //创建一个矩阵，矩阵中每个元素都是单通道的8UC1,相当于一张黑图
    //edge_mask(cv::Rect(520, 205, 300, 250))=1;
    edge_mask(cv::Rect(0, 0, img.cols,
                       img.rows)) = 1;         //CvRect（含4个数据成员，x、y、width、height）是OpenCV里面的基本数据类型，其功能是包通过定义矩形左上角坐标和矩形的宽和高来确定一个矩形。把矩形里的元素都变1，也就是全变1了

    // Mat roi = img1(Rect(0, 0, img2.cols, img2.rows));     可以见得roi是img1图像的一部分，还是从左上角开始，长宽等于img2的一部分。



    img.copyTo(edge_mask, edge_mask);
    //pcl::io::savePCDFileASCII ("/home/vishnu/final1.pcd", scan.point_cloud);
//    image.copyTo(imageROI，mask);
//    则是不仅把image这张图复制（copy to）到mageROI上，且image对应mask中像素值为0的像素点都不会贴到imageROI上。



    img = edge_mask;      //这波对黑图的作作没没看太懂，但不重要

    //cv:imwrite("/home/vishnu/marker.png", edge_mask);

    pcl::PointCloud<pcl::PointXYZ> pc = scan;      //把点云用新变量PC存起来
    //scan = Velodyne::Velodyne(filtered_pc);

    cv::Rect frame(0, 0, img.cols, img.rows);    //又画了个框框

    //pcl::io::savePCDFileASCII("/home/vishnu/final2.pcd", scan.point_cloud);

    cv::Mat image_edge_laser = project(P, frame, scan, NULL);      ///utils.h里定义了 //投影雷达点，3D变2D，仅为画图用，创建关联点对数据在后面

//    cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud<pcl::PointXYZ> point_cloud, pcl::PointCloud<pcl::PointXYZ> *visible_points)
//    {
//        cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);
//
//        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)    这里就是对点云一一处理
//        {
//
//            // behind the camera
//            if (pt->z < 0)
//            {
//                continue;               //直接下一次FOR循环    //把向后方的点云全部砍掉了
//            }
//
//            //float intensity = pt->intensity;
//            cv::Point xy = project(*pt, projection_matrix);

////////////////////////
//PROJECT3D变2D的函数定义 在UTILS。H里
//    cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix)
//{
//        //cv::Point2f xy = projectf(pt, projection_matrix);
    //      cv::Mat pt_3D(4, 1, CV_32FC1);
//
    //      pt_3D.at<float>(0) = pt.x;
    //      pt_3D.at<float>(1) = pt.y;
    //      pt_3D.at<float>(2) = pt.z;
    //      pt_3D.at<float>(3) = 1.0f;
    //      cv::Mat pt_2D = projection_matrix * pt_3D;      //用相机的内参阵阵去乘它，四维变三维
    //      float w = pt_2D.at<float>(2);
    //      float x = pt_2D.at<float>(0) / w;
    //      float y = pt_2D.at<float>(1) / w;
    //      return cv::Point(x, y);
    //  }
//////////////////////////


//            if (xy.inside(frame))                    保证三维投影点在照片尺寸范围内
//            {
//                if (visible_points != NULL)
//                {
//                    visible_points->push_back(*pt);
//                }
//
//                //cv::circle(plane, xy, 3, intensity, -1);
//                //plane.at<float>(xy) = intensity;
//                plane.at<float>(xy)=250;                      //在PLANE这个图里一个个画点
//            }
//        }
//
//        cv::Mat plane_gray;
//        cv::normalize(plane, plane_gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);     //归一化PLANE到0-255输出到PLANEGRAY
//        cv::dilate(plane_gray, plane_gray, cv::Mat());                           //把那个点彭胀起来，是是是变为几个圆点，这个图就是PLANE-GRAY
//
//        return plane_gray;
//    }





    cv::threshold(image_edge_laser, image_edge_laser, 10, 255, 0);      //image_edge_laser是关键，大于10的都给弄成白色,是为了dialate吧

    //   形式：void cvThreshold( const CvArr* src, CvArr* dst, double threshold, double max_value, int threshold_type );
    //   　　src：原始数组 (单通道 , 8-bit of 32-bit 浮点数)。dst：输出数组，必须与 src 的类型一致，或者为 8-bit。
    //  　　threshold：阈值
    // 　　max_value：使用 CV_THRESH_BINARY 和 CV_THRESH_BINARY_INV 的最大值。
    //  　　threshold_type：阈值类型 threshold_type=CV_THRESH_BINARY:
    // 　　如果 src(x,y)>threshold 0,dst(x,y) = max_value, 否则.置0
    //  　　threshold_type=CV_THRESH_BINARY_INV:
    //  　　如果 src(x,y)>threshold,dst(x,y) = 0; 否则,dst(x,y) = max_value.




    cv::Mat combined_rgb_laser;
    std::vector<cv::Mat> rgb_laser_channels;

    rgb_laser_channels.push_back(image_edge_laser);                        //往rgb_laser_channels里塞了三个MAT 也即三个通道
    rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
    rgb_laser_channels.push_back(img);

    cv::merge(rgb_laser_channels,
              combined_rgb_laser);           //将image_edge_laser，cv::Mat::zeros(image_edge_laser.size()，img三通道合并，输出
    /*cv::namedWindow("combined", cv::WINDOW_NORMAL);
    cv::imshow("combined", combined_rgb_laser);
    cv::waitKey(5);
    */
///////////////////////////////////////////////////////////////以上出图完成*****************************************/////////////////////////////////////////
///////////////////////////////////////////////////////////////以上出图完成*****************************************/////////////////////////////////////////
///////////////////////////////////////////////////////////////以上出图完成*****************************************/////////////////////////////////////////
///////////////////////////////////////////////////////////////以上出图完成*****************************************/////////////////////////////////////////




    std::map<std::pair<int, int>, std::vector<float> > c2D_to_3D;       //MAP可以认为就是一个容器，两两存储，也是创建一个链表
    std::vector<float> point_3D;

    /* store correspondences */
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc.points.begin();
         pt < pc.points.end(); pt++)        //PC即为PCL格式点云
    {

        // behind the camera
        if (pt->z < 0) {
            continue;        //把向后方的点云全部砍掉了，没用
        }

        cv::Point xy = project(*pt, P);       //投影雷达点，3D变2D
        if (xy.inside(frame)) {
            //create a map of 2D and 3D points         应该是2D 3D点一一对应的表    c2D_to_3D很重要，是创建的2D对应3D的点关联对！！！！！！！在之后框点后能对应到3D点
            point_3D.clear();
            point_3D.push_back(pt->x);
            point_3D.push_back(pt->y);
            point_3D.push_back(pt->z);
            c2D_to_3D[std::pair<int, int>(xy.x, xy.y)] = point_3D;
        }
    }

    /* print the correspondences */
    /*for(std::map<std::pair<int, int>, std::vector<float> >::iterator it=c2D_to_3D.begin(); it!=c2D_to_3D.end(); ++it)
    {
        std::cout << it->first.first << "," << it->first.second << " --> " << it->second[0] << "," <<it->second[1] << "," <<it->second[2] << "\n";
    }*/

    /* get region of interest */

    const int QUADS = num_of_markers;      //板子的数量
    std::vector<int> LINE_SEGMENTS(1, 10); //assuming each has 4 edges and 4 corners       QUADS个元素，且值均为8,意思是有QUADS个marker,每个有四条边，给下面循环用

    pcl::PointCloud<pcl::PointXYZ>::Ptr board_corners(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr marker(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> c_3D;      //3D点向量存储
    std::vector<cv::Point2f> c_2D;      //2D点向量存储


    cv::namedWindow("cloud", cv::WINDOW_NORMAL);            //开开两个窗口
    cv::namedWindow("polygon", cv::WINDOW_NORMAL);
    //cv::namedWindow("combined", cv::WINDOW_NORMAL);

    std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");             //获取ROS包的绝对路径
    std::ofstream outfile(pkg_loc + "/conf/points.txt", std::ios_base::trunc);
    outfile << QUADS * 9 << "\n";                  //往这个POINTS。TXT里写东西，写边数，之后的角点坐标都要往里头写！！find_transform要读


    std::ofstream outfile2(pkg_loc + "/conf/multiplepointsransac.txt", std::ios_base::trunc);


/////////////////////////////////////////////////////////////////////////*************从这儿开始大改了*************************************************////////////////
/////////////////////////////////////////////////////////////////////////*************从这儿开始大改了*************************************************////////////////
/////////////////////////////////////////////////////////////////////////*************从这儿开始大改了*************************************************////////////////
    //for(int q=0; q<QUADS; q++)
//	{
//		std::cout << "---------Moving on to next marker--------\n";
    std::vector<Eigen::VectorXf> line_model;
    std::vector<Eigen::VectorXf> line_modelmultiple;
    for (int i = 0; i < 10; i++)         //一条边循环一次
    {
        cv::Point _point_;
        std::vector<cv::Point> polygon;
        int collected;

        // get markings in the first iteration only
        if (iteration_count == 0) {
            polygon.clear();
            collected = 0;
            while (collected != 4)     //左键后Q
            {

                cv::setMouseCallback("cloud", onMouse, &_point_);         //鼠标点击选择区域！！！这里只是画一个框，循环四次

                cv::imshow("cloud",
                           image_edge_laser);      //把之前混搭在一起的图显示出来！！！image_edge_laser    combined_rgb_laser的作用是显示出画线的结果
                cv::waitKey(0);
                ++collected;
                //std::cout << _point_.x << " " << _point_.y << "\n";
                polygon.push_back(_point_);   //把鼠标点到的点放到POLYGON里
            }
            stored_corners.push_back(polygon);      //再存到STORED-CORNERS里
        }

        polygon = stored_corners[10 * 0 + i];  //更新POLYGON

        cv::Mat polygon_image = cv::Mat::zeros(image_edge_laser.size(), CV_8UC1);

        rgb_laser_channels.clear();                          //更新存储三张用与混搭图的向量
        rgb_laser_channels.push_back(image_edge_laser);                        //原来的图
        rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
        rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
        cv::merge(rgb_laser_channels, combined_rgb_laser);                //combined_rgb_laser的作用是显示出画线的结果，画了四条线后更新一下

        for (int j = 0; j < 4; j++)              //J就是画四条线框住
        {
            cv::line(combined_rgb_laser, polygon[j], polygon[(j + 1) % 4], cv::Scalar(0, 255, 0));       //在图上画框住的线条
        }

        // initialize PointClouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);      //创建两个新的点云变量
        pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::map<std::pair<int, int>, std::vector<float> >::iterator it = c2D_to_3D.begin();
             it != c2D_to_3D.end(); ++it)     //IDE有问题有用代码是灰的    c2D_to_3D很重要，是之前创建的2D对应3D的点关联对！！！！！！！
        {

            if (cv::pointPolygonTest(cv::Mat(polygon), cv::Point(it->first.first, it->first.second), true) >
                0)     //cv::pointPolygonTest用于判断一个点是否在轮廓中
            {
                cloud->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                               it->second[2]));                  //将被框住的点云XYZ存到CLOUD里！！！！！！

                switch(i)
                {
                    case 0:
                        pointclouds1->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 1:
                        pointclouds2->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 2:
                        pointclouds3->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 3:
                        pointclouds4->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 4:
                        pointclouds5->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 5:
                        pointclouds6->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 6:
                        pointclouds7->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 7:
                        pointclouds8->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 8:
                        pointclouds9->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的
                    case 9:
                        pointclouds10->push_back(pcl::PointXYZ(it->second[0], it->second[1],
                                                              it->second[2]));
                        break; // 可选的




//                    // 您可以有任意数量的 case 语句
//                default : // 可选的
//                    statement(s);
                }


                rectangle(combined_rgb_laser, cv::Point(it->first.first, it->first.second),
                          cv::Point(it->first.first, it->first.second), cv::Scalar(0, 0, 255), 3, 8, 0); // RED point
            }
        }


        cv::imshow("polygon", combined_rgb_laser);       //画个图来来告诉你框的结果
        cv::waitKey(4);

        //pcl::io::savePCDFileASCII("/home/vishnu/line_cloud.pcd", *cloud);



        std::vector<int> inliers;
        Eigen::VectorXf model_coefficients;           //线模型系数


        // created RandomSampleConsensus object and compute the appropriated model
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(
                new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));     //cloud就是上面被框住的2D后转为3D的点云

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);          //RANSAC拟合直线！！！！
        ransac.setDistanceThreshold(0.01);                    //RANSAC
        ransac.computeModel();           //RANSAC
        ransac.getInliers(inliers);     //RANSAC
        ransac.getModelCoefficients(model_coefficients);     //RANSAC
        line_model.push_back(model_coefficients);                 //拟合结果放到里面

        std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示
        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
        //pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_line_cloud.pcd", *final);
        *marker += *final;              //至此，一条边解算完成，下下来到下一条边的循环





//            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);          //RANSAC拟合直线！！！！
//            ransac.setDistanceThreshold(0.01);                    //RANSAC
//            ransac.computeModel();           //RANSAC
//            ransac.getInliers(inliers);     //RANSAC
//            ransac.getModelCoefficients(model_coefficients);     //RANSAC
//            line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面
//
//            std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示
//            // copies all inliers of the model computed to another PointCloud
//            pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
//            //pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_line_cloud.pcd", *final);
//            *marker += *final;              //至此，一条边解算完成，下下来到下一条边的循环





        }






/////////////////// //至此，一条边解算完成，接下来到下一条边的循环



// if(iteration_count==MAX_ITERS-1)            //开始做多次打点RANSAC的工作
if(iteration_count==MAX_ITERS-1)
{


     std::vector<int> inliers;
Eigen::VectorXf model_coefficients;           //线模型系数


// created RandomSampleConsensus object and compute the appropriated model


pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l1(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds1));


pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l2(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds2));

pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l3(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds3));

pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l4(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds4));

pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l5(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds5));

pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l6(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds6));

pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l7(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds7));

pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l8(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds8));

pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l9(
        new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds9));
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l10(
            new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pointclouds10));





pcl::RandomSampleConsensus<pcl::PointXYZ> ransac0(model_l1);          //RANSAC拟合直线！！！！
ransac0.setDistanceThreshold(0.01);                    //RANSAC
ransac0.computeModel();           //RANSAC
ransac0.getInliers(inliers);     //RANSAC
ransac0.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示





pcl::RandomSampleConsensus<pcl::PointXYZ> ransac1(model_l2);          //RANSAC拟合直线！！！！
ransac1.setDistanceThreshold(0.01);                    //RANSAC
ransac1.computeModel();           //RANSAC
ransac1.getInliers(inliers);     //RANSAC
ransac1.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示



pcl::RandomSampleConsensus<pcl::PointXYZ> ransac2(model_l3);          //RANSAC拟合直线！！！！
ransac2.setDistanceThreshold(0.01);                    //RANSAC
ransac2.computeModel();           //RANSAC
ransac2.getInliers(inliers);     //RANSAC
ransac2.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示




pcl::RandomSampleConsensus<pcl::PointXYZ> ransac3(model_l4);          //RANSAC拟合直线！！！！
ransac3.setDistanceThreshold(0.01);                    //RANSAC
ransac3.computeModel();           //RANSAC
ransac3.getInliers(inliers);     //RANSAC
ransac3.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示





pcl::RandomSampleConsensus<pcl::PointXYZ> ransac4(model_l5);          //RANSAC拟合直线！！！！
ransac4.setDistanceThreshold(0.01);                    //RANSAC
ransac4.computeModel();           //RANSAC
ransac4.getInliers(inliers);     //RANSAC
ransac4.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示





pcl::RandomSampleConsensus<pcl::PointXYZ> ransac5(model_l6);          //RANSAC拟合直线！！！！
ransac5.setDistanceThreshold(0.01);                    //RANSAC
ransac5.computeModel();           //RANSAC
ransac5.getInliers(inliers);     //RANSAC
ransac5.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示





pcl::RandomSampleConsensus<pcl::PointXYZ> ransac6(model_l7);          //RANSAC拟合直线！！！！
ransac6.setDistanceThreshold(0.01);                    //RANSAC
ransac6.computeModel();           //RANSAC
ransac6.getInliers(inliers);     //RANSAC
ransac6.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示





pcl::RandomSampleConsensus<pcl::PointXYZ> ransac7(model_l8);          //RANSAC拟合直线！！！！
ransac7.setDistanceThreshold(0.01);                    //RANSAC
ransac7.computeModel();           //RANSAC
ransac7.getInliers(inliers);     //RANSAC
ransac7.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示





pcl::RandomSampleConsensus<pcl::PointXYZ> ransac8(model_l9);          //RANSAC拟合直线！！！！
ransac8.setDistanceThreshold(0.01);                    //RANSAC
ransac8.computeModel();           //RANSAC
ransac8.getInliers(inliers);     //RANSAC
ransac8.getModelCoefficients(model_coefficients);     //RANSAC
line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示


    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac9(model_l10);          //RANSAC拟合直线！！！！
    ransac9.setDistanceThreshold(0.01);                    //RANSAC
    ransac9.computeModel();           //RANSAC
    ransac9.getInliers(inliers);     //RANSAC
    ransac9.getModelCoefficients(model_coefficients);     //RANSAC
    line_modelmultiple.push_back(model_coefficients);                 //拟合结果放到里面

    std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";         //拟合参数显示









//                    // 您可以有任意数量的 case 语句
//                default : // 可选的
//                    statement(s);
}











		
		/* calculate approximate intersection of lines */  // 利用上面的RANSAC结果算交点
		

		Eigen::Vector4f p1, p2, p_intersect;
		pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud<pcl::PointXYZ>);    //定义交点点云
		for(int i=0; i<LINE_SEGMENTS[0]-1; i++)                            //对每条边
		{
			pcl::lineToLineSegment(line_model[i], line_model[(i+1)%LINE_SEGMENTS[0]], p1, p2);     //计算空间直线I和空间直线I+1的最小公垂线进而计算交点

           // PCL_EXPORTS void pcl::lineToLineSegment	(const Eigen::VectorXf &  line_a,
           //                                             const Eigen::VectorXf &  line_b,
           //                                             Eigen::Vector4f & 	pt1_seg,
           //                                             Eigen::Vector4f & 	pt2_seg
           // )
           // Parameters:
           // line_a 	the coefficients of the first line (point, direction)
           // line_b 	the coefficients of the second line (point, direction)
           // pt1_seg 	the first point on the line segment                  P1，P2是四维度的齐次坐标向量
           // pt2_seg 	the second point on the line segment

			for(int j=0; j<4; j++)
			{
				p_intersect(j) = (p1(j) + p2(j))/2.0;                         //交点就认为在最小公垂线的中点，四循环是因为四维度 Eigen::Vector4f
			}
			c_3D.push_back(cv::Point3f(p_intersect(0), p_intersect(1), p_intersect(2)));         //把齐次坐标前三维存到向量里备用c_3D，后面肯定会用到，第四维是1吧，没用
			corners->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));    //把齐次坐标前三维存到点云里备用corners，但这个点云它不是向量啊，怎么更新呢？
			std::cout << "Point of intersection is approximately: \n" << p_intersect << "\n";
			//std::cout << "Distance between the lines: " << (p1 - p2).squaredNorm () << "\n";
			std::cout << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) <<  "\n";       //输出点东西
			outfile << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) <<  "\n";   ///把结果也就是交点坐标写到file里，到时候find_transform读那个文件






            switch(saveflag)
            {
                case 1:
                    savepoints1->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 2:
                    savepoints2->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 3:
                    savepoints3->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 4:
                    savepoints4->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 5:
                    savepoints5->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 6:
                    savepoints6->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 7:
                    savepoints7->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 8:
                    savepoints8->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的
                case 9:
                    savepoints9->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
                    break; // 可选的




//                    // 您可以有任意数量的 case 语句
//                default : // 可选的
//                    statement(s);
            }

            saveflag++;
            if(saveflag==10)
            {
                saveflag=1;

            }




        }
		
		*board_corners += *corners;

		std::cout << "Distance between the corners:\n";
		for(int i=0; i<9; i++)
		{
			std::cout << 

						sqrt(                                                  //SQRT是求平方根，POW是求平方根，如POW（2，4）为2的4次方
						  pow(c_3D[8*0+i].x - c_3D[8*0+(i+1)%8].x, 2)          //Q是第几块标定板，总共输出4个数，分别为交点和下一个交点的欧式距离，没卵用
						+ pow(c_3D[8*0+i].y - c_3D[8*0+(i+1)%8].y, 2)
						+ pow(c_3D[8*0+i].z - c_3D[8*0+(i+1)%8].z, 2)
						)

						<< std::endl;
		}

	//}
	outfile.close();

	iteration_count++;            //迭代次数计数，运行多少次后就关闭ROS
    if(iteration_count==MAX_ITERS)
    {
        outfile2 <<9<< "\n";


        for(int i=0; i<LINE_SEGMENTS[0]-1; i++)                            //对每条边
        {
            pcl::lineToLineSegment(line_modelmultiple[i], line_modelmultiple[(i + 1) % LINE_SEGMENTS[0]], p1,
                                   p2);     //计算空间直线I和空间直线I+1的最小公垂线进而计算交点

            // PCL_EXPORTS void pcl::lineToLineSegment	(const Eigen::VectorXf &  line_a,
            //                                             const Eigen::VectorXf &  line_b,
            //                                             Eigen::Vector4f & 	pt1_seg,
            //                                             Eigen::Vector4f & 	pt2_seg
            // )
            // Parameters:
            // line_a 	the coefficients of the first line (point, direction)
            // line_b 	the coefficients of the second line (point, direction)
            // pt1_seg 	the first point on the line segment                  P1，P2是四维度的齐次坐标向量
            // pt2_seg 	the second point on the line segment

            for (int j = 0; j < 4; j++) {
                p_intersect(j) = (p1(j) + p2(j)) /
                                 2.0;                         //交点就认为在最小公垂线的中点，四循环是因为四维度 Eigen::Vector4f
            }

            std::cout << "开始写入多次打点RANSAC结果" << std::endl;
            std::cout << "multipleransac Point of intersection is approximately: \n" << p_intersect << "\n";
            //std::cout << "Distance between the lines: " << (p1 - p2).squaredNorm () << "\n";
            std::cout << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) << "\n";       //输出点东西
            outfile2 << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2)
                    << "\n";   ///把结果也就是交点坐标写到file里，到时候find_transform读那个文件





        }

        outfile2.close();













        std::ofstream outfile(pkg_loc + "/conf/averagepoints.txt", std::ios_base::trunc);

        std::cout << "开始做点平均处理" << std::endl;


        outfile <<9<< "\n";


        float counttemp1 = 0;
        float xtemp1 = 0;
        float ytemp1 = 0;
        float ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints1).begin(); pt1 < (*savepoints1).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }


        counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints2).begin(); pt1 < (*savepoints2).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }


        counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints3).begin(); pt1 < (*savepoints3).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }

        counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints4).begin(); pt1 < (*savepoints4).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }

        counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints5).begin(); pt1 < (*savepoints5).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }

        counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints6).begin(); pt1 < (*savepoints6).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }

        counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints7).begin(); pt1 < (*savepoints7).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }

        counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints8).begin(); pt1 < (*savepoints8).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }
 counttemp1 = 0;
        xtemp1 = 0;
        ytemp1 = 0;
        ztemp1 = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator pt1 = (*savepoints9).begin(); pt1 < (*savepoints9).end(); pt1++) {

            xtemp1 = xtemp1 + pt1->x;          //先把圈到的点全都加在一起
            ytemp1 = ytemp1 + pt1->y;
            ztemp1 = ztemp1 + pt1->z;
            counttemp1++;       //计算有多少个点
        }
        if(counttemp1)
        {
            xtemp1=xtemp1/counttemp1;
            ytemp1=ytemp1/counttemp1;
            ztemp1=ztemp1/counttemp1;

            outfile << xtemp1<<" " <<ytemp1<<" "<<ztemp1<< "\n";
        }
        else
        {
            outfile <<0<<" " <<0<<" "<<0<< "\n";
        }



        outfile.close();

    }
	if(iteration_count == MAX_ITERS)
	{
		ros::shutdown();
	}
	/* store point cloud with intersection points */
	//pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_marker.pcd", *marker);
	//pcl::io::savePCDFileASCII("/home/vishnu/RANSAC_corners.pcd", *board_corners);
}
