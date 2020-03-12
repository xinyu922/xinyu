#include <iostream>
#include <sl/Camera.hpp>
#include <time.h>

#include <opencv2/opencv.hpp>
#include "MarkerDetection.h"
#include <fstream>
#include <iomanip>
#include <chrono>

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

    std::string data_dir="/home/bdi048/ws/SDK_depth_get_xinyu/SDK_exe/" + s_tmp + "_zed_image_data/";

    //创建文件夹

    int flag=mkdir(data_dir.c_str(), 0777); 

    //
    ofstream outfile;
    outfile.open("zed_depth.csv");
    outfile << "time," << "u," << "v," << "depth," << "\n";
    roiRegion.resize(4);
    CMarkerDetection markDetector;

    Camera zed;
    InitParameters init_params;
    init_params.depth_mode = DEPTH_MODE_ULTRA;
    init_params.coordinate_units = UNIT_METER;

    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;
    sl::Mat image, depth, point_cloud;

    int counter = 0;

    while (true) {
        for (int i=0; i<4; i++)
        {
            roiRegion[i] = 0;
        }
        if (zed.grab(runtime_parameters) == SUCCESS) {
            zed.retrieveImage(image, VIEW_LEFT);
            zed.retrieveMeasure(depth, MEASURE_DEPTH);
            zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA);
            cv::Mat colorImg = cv::Mat((int)image.getHeight(), (int)image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM_CPU));

            markDetector.MarkerDetection(&colorImg, roiRegion, 100);
            int x = (roiRegion[0] + roiRegion[1]) / 2;
            int y = (roiRegion[2] + roiRegion[3]) / 2;
            if (x == 0 || y == 0)
            {
                cv::imshow("colorImg",colorImg);
                cv::waitKey(1);
                char key = static_cast<char>(cv::waitKey(1));
                if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
                    break;
                }
                continue;
            }
                
            cv::circle(colorImg, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 3);
            sl::float1 depth_value;
            depth.getValue(x, y, &depth_value);

	        string str_distance = to_string(depth_value);
      	    cv::putText(colorImg, str_distance+'m', cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 3);

            double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            now = now / 1000;
            outfile << std::setprecision(20) << now << "," << x << "," << y << "," << depth_value << std::endl;
            
            std::string img_path = data_dir + to_string(counter) + ".png";
            cv::imwrite(img_path, colorImg);
            counter++;

            cv::imshow("colorImg", colorImg);
            cv::waitKey(1);

	        char key = static_cast<char>(cv::waitKey(1));
    	    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      		break;}
        }
    }
}
