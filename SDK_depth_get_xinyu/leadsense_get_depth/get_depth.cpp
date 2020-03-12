#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

// EvoBinSDK header
#include "evo_depthcamera.h"
#include "evo_matconverter.h"

#include "MarkerDetection.h"

#include <fstream>
#include <time.h>
#include <iomanip>
#include <chrono>

evo::bino::DepthCamera camera;
std::vector<int> roiRegion;

int main(int argc, char** argv)
{
    ofstream outfile;
    outfile.open("leadSense_depth.csv");
    outfile << "time," << "u," << "v," << "depth," << "\n";
    time_t timep;

    // open camera
    roiRegion.resize(4);
    evo::bino::RESOLUTION_FPS_MODE res_mode = evo::bino::RESOLUTION_FPS_MODE_HD720_60;
    evo::RESULT_CODE res = camera.open(res_mode);
    std::cout << "depth camera open: " << result_code2str(res) << std::endl;
    //grab parameters
    evo::bino::GrabParameters grab_parameters;


    CMarkerDetection markDetector;
    if (res == evo::RESULT_CODE_OK)
    {
        //evo Mat
        evo::Mat<unsigned char> evo_image;
        evo::Mat<float> evo_distance;
        //cv Mat
        cv::Mat cv_image, cv_image_bgr, cv_depth, cv_depth_bgr;

        while(true)
        {
            for (int i=0; i<4; i++)
            {
                roiRegion[i] = 0;
            }
            if (camera.grab(grab_parameters) == evo::RESULT_CODE_OK)
            {
                evo_image = camera.retrieveView(evo::bino::VIEW_TYPE_LEFT, evo::MAT_TYPE_CPU);
                evo_distance = camera.retrieveDepth(evo::bino::DEPTH_TYPE_DISTANCE_XYZ, evo::MAT_TYPE_CPU);
            }
            int w_ = evo_distance.getWidth();
            int h_ = evo_distance.getHeight();

            cv_image = evo::evoMat2cvMat(evo_image);
            cv::cvtColor(cv_image, cv_image_bgr, CV_RGBA2BGR);
            markDetector.MarkerDetection(&cv_image_bgr, roiRegion, 100);
            int x = (roiRegion[0] + roiRegion[1]) / 2;
            int y = (roiRegion[2] + roiRegion[3]) / 2;
            if(x == 0 || y == 0)
            {
                cv::imshow("colorImg", cv_image_bgr);
                cv::waitKey(1);
                // char key = static_cast<char>(cv::waitKey(1));
                // if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
                //     break;}
                // continue;
            }
            else
            {
                cv::circle(cv_image_bgr, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 3);

                cv::Point3d p;
                p.x = evo_distance.getValue(x,y,1)/1000;
                p.y = evo_distance.getValue(x,y,2)/1000;
                p.z = evo_distance.getValue(x,y,3)/1000;

                // float depth_value = evo_distance.getValue(x, y, 3)/1000;
                float depth_value = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
                string str_distance = to_string(depth_value);

                cv::putText(cv_image_bgr, str_distance+'m', cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 0, 255), 3);

                double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                now = now/1000;
                outfile << std::setprecision(20) << now << "," << x << "," << y << "," << depth_value << std::endl;
    //            std::cout << evo_distance.getValue(640, 360) << std::endl;
                cv::imshow("colorImg", cv_image_bgr);

                cv::waitKey(1);

                char key = static_cast<char>(cv::waitKey(1));
                if (key == 27 || key == 'q' || key == 'Q') 
                {  // ESC/Q
                    break;
                }
            }
            
        }
    }
    cv::destroyAllWindows();
}
