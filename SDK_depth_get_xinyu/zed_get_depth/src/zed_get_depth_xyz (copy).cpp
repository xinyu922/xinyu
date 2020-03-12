#include <iostream>
#include <sl/Camera.hpp>
#include <time.h>

#include <opencv2/opencv.hpp>
#include "MarkerDetection.h"
#include <fstream>
#include <iomanip>
#include <chrono>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>

using namespace sl;
std::vector<int> roiRegion;

int main(int argc, char** argv)
{
    //创建文件夹保存识别到的mark的图像
    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&timep) );
    string s_tmp(&tmp[0],&tmp[strlen(tmp)]);

    //std::string data_dir="zed/" + s_tmp + "_zed_image_data/";
    std::string data_dir= s_tmp + "_zed_image_data/";
    //创建文件夹

    int flag=mkdir(data_dir.c_str(), 0777); 

    //
    ofstream outfile;
    outfile.open("zed_depth.csv");
    outfile << "time," << "u," << "v," << "depth," << "\n";
    roiRegion.resize(4);
    CMarkerDetection markDetector;

    Camera zed;
    // zed.setDepthMaxRangeValue(40);
    InitParameters init_params;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;
    // init_params.camera_resolution = RESOLUTION_HD1080; // Use HD1080 video mode
    // init_params.camera_fps = 30;    

    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_FILL;
    sl::Mat image_l, image_r, depth, point_cloud;

    int counter = 0;

    while (true) 
    {
        for (int i=0; i<4; i++)
        {
            roiRegion[i] = 0;
        }
        if (zed.grab(runtime_parameters) == SUCCESS) 
        {
            zed.retrieveImage(image_l, VIEW_LEFT);
            // zed.retrieveImage(image_r, VIEW_RIGHT);
            // zed.retrieveMeasure(depth, MEASURE_DEPTH);
            zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA);
            cv::Mat image_left = cv::Mat((int)image_l.getHeight(), (int)image_l.getWidth(), CV_8UC4, image_l.getPtr<sl::uchar1>(sl::MEM_CPU));
            // cv::Mat image_right = cv::Mat((int)image_r.getHeight(), (int)image_r.getWidth(), CV_8UC4, image_r.getPtr<sl::uchar1>(sl::MEM_CPU));
            // cv::Mat image_depth = cv::Mat((int)depth.getHeight(), (int)depth.getWidth(), CV_8UC4, depth.getPtr<sl::uchar1>(sl::MEM_CPU));

            cv::Mat colorImg;
            image_left.copyTo(colorImg);

            markDetector.MarkerDetection(&colorImg, roiRegion, 100);
            int x = (roiRegion[0] + roiRegion[1]) / 2;
            int y = (roiRegion[2] + roiRegion[3]) / 2;
            if (x == 0 || y == 0)
            {
                cv::imshow("ZedImg",colorImg);
                cv::waitKey(1);
                // char key = static_cast<char>(cv::waitKey(1));
                // if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
                //     break;
                // }
            }
            else
            {
                cv::circle(colorImg, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 3);

                // sl::float1 depth_value;
                // depth.getValue(x, y, &depth_value);
                // string str_distance = to_string(depth_value);

                //
                sl::float4 point3D;
                // Measure the distance of a point in the scene represented by pixel (i,j)
                point_cloud.getValue(x,y,&point3D);
                float distance = sqrt(point3D.x*point3D.x + point3D.y*point3D.y + point3D.z*point3D.z);
                string str_distance = to_string(distance);

                cv::putText(colorImg, str_distance+'m', cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 3);

                double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                now = now / 1000;
                outfile << std::setprecision(20) << now << "," << x << "," << y << "," << distance << std::endl;
                
                std::string img_mark_path = data_dir + "mark/" + to_string(counter) + ".png";
                std::string img_left_path = data_dir + "left/" + to_string(counter) + ".png";
                std::string img_right_path = data_dir + "right/" + to_string(counter) + ".png";
                std::string img_depth_path = data_dir + "depth/" + to_string(counter) + ".png";

                cv::imwrite(img_mark_path, colorImg);
                cv::imwrite(img_left_path, image_left);
                cv::imwrite(img_right_path, image_right);
                cv::imwrite(img_depth_path, image_depth);
                counter++;

                cv::imshow("ZedImg", colorImg);
                cv::waitKey(1);

                // char key = static_cast<char>(cv::waitKey(1));
                // if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
                // break;}
                }  
        }
    }
}
