#include <iostream>
#include <sl/Camera.hpp>
#include <time.h>

#include <opencv2/opencv.hpp>
#include "MarkerDetection.h"

#include <iomanip>
#include <chrono>

using namespace sl;
std::vector<int> roiRegion;

int main(int argc, char** argv)
{
    ofstream outfile;
    outfile.open("zed_depth.csv");
    outfile << "time," << "u," << "v," << "depth," << "\n";
    roiRegion.resize(4);
    CMarkerDetection markDetector;

    Camera zed;
    InitParameters init_params;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;

    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;
    sl::Mat image, depth, point_cloud;
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

            cv::imshow("colorImg", colorImg);
            cv::waitKey(1);

	    char key = static_cast<char>(cv::waitKey(1));
    	    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      		break;
    }
        }
    }
}
