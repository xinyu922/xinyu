#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <thread> //

// EvoBinSDK header
#include "evo_depthcamera.h"
#include "evo_matconverter.h"

#include <fstream>
#include <time.h>
#include <iomanip>
#include <chrono>
#include <sys/types.h>
#include <sys/stat.h>

evo::bino::DepthCamera camera;
using namespace std;
// using namespace sl;

bool running;


void handleKey(char key)
{
    switch (key)
    {
    case 27:
        running = false;
        break;
    default:
        break;
    }
}

void imuPoll(std::string imu_txt)
{
    std::ofstream file_imu;
    file_imu.open(imu_txt); //
    evo::RESULT_CODE res;
    while(true)
    {
        if (res == evo::RESULT_CODE_OK)
        {
            evo::imu::IMU_DATA_TYPE data_type = evo::imu::IMU_DATA_TYPE_RAW; //   
            // Set IMU data type
            camera.setIMUDataType(data_type);
            
            // Start retrieve IMU data
            res = camera.startRetrieveIMU();
            std::vector<evo::imu::IMUData> vector_data = camera.retrieveIMUData();

            double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            now = now/1000;

            // Print newest result
            if (vector_data.size() > 0)
            {
                evo::imu::IMUData data = vector_data.at(vector_data.size() - 1);
                if (data_type == evo::imu::IMU_DATA_TYPE_RAW)
                {
                    file_imu << std::setprecision(4) << std::fixed << "accel/gyro/magnet/time:\t"
                        << data.accel[0] << " " << data.accel[1] << " " << data.accel[2] << "\t"
                        << data.gyro[0] << " " << data.gyro[1] << " " << data.gyro[2] << "\t"
                        << data.mag[0] << " " << data.mag[1] << " " << data.mag[2] << "\t"
                        << std::setprecision(10) << now
                        << std::endl;
                    // std::cout << std::setprecision(4) << std::fixed << "accel/gyro/magnet/time:\t"
                    //     << data.accel[0] << " " << data.accel[1] << " " << data.accel[2] << "\t"
                    //     << data.gyro[0] << " " << data.gyro[1] << " " << data.gyro[2] << "\t"
                    //     << data.mag[0] << " " << data.mag[1] << " " << data.mag[2] << "\t"
                    //     << std::setprecision(20) << now
                    //     << std::endl;	
                }
                
            }
            // now = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = now - start;
        }
    }
}

int main(int argc, char** argv)
{

    time_t timep;
    time (&timep);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&timep) );
    string s_tmp(&tmp[0],&tmp[strlen(tmp)]);

    std::string data_dir="/home/dzsb078/Documents/" + s_tmp + "_Stereo_Leadsence_data/";

    //创建Leadsence_data文件夹

    int flag=mkdir(data_dir.c_str(), 0777); 

    //创建子文件夹

    std::string data_dir_image_0= data_dir + "image_0/";
    std::string data_dir_image_1= data_dir + "image_1/";
    std::string data_dir_image_depth= data_dir + "depth/";
    
    int flag_1=mkdir(data_dir_image_0.c_str(), 0777);
    int flag_2=mkdir(data_dir_image_1.c_str(), 0777);
    int flag_5=mkdir(data_dir_image_depth.c_str(), 0777);

    string time_txt = data_dir + "times.txt";
    string imu_txt = data_dir + "imu.txt"; // 

    ofstream file_obj;
    file_obj.open(time_txt);
    int counter = 0;

    evo::bino::RESOLUTION_FPS_MODE res_mode = evo::bino::RESOLUTION_FPS_MODE_HD720_60;
    evo::RESULT_CODE res = camera.open(res_mode);

    std::cout << "depth camera open: " << result_code2str(res) << std::endl;
    //grab parameters
    evo::bino::GrabParameters grab_parameters;
    // grab_parameters.do_rectify = false;

    evo::Mat<unsigned char> evo_image_left, evo_image_right, evo_image_depth;
    // evo::Mat<float> evo_image_depth;  ///
    //cv Mat
    cv::Mat cv_image_left, cv_image_right, cv_image_depth;

    // If successed
    if (res == evo::RESULT_CODE_OK)
    {
        std::thread imu_poll(imuPoll, imu_txt);

        while (true)
        {
            
            if (camera.grab(grab_parameters) == evo::RESULT_CODE_OK)
            {
                evo_image_left = camera.retrieveView(evo::bino::VIEW_TYPE_LEFT, evo::MAT_TYPE_CPU);
                evo_image_right = camera.retrieveView(evo::bino::VIEW_TYPE_RIGHT, evo::MAT_TYPE_CPU);
                evo_image_depth = camera.retrieveNormalizedDepth(evo::bino::DEPTH_TYPE_DISTANCE_Z, evo::MAT_TYPE_CPU, 255, 0); //
            }

            cv_image_left = evo::evoMat2cvMat(evo_image_left);
            cv_image_right = evo::evoMat2cvMat(evo_image_right);
            cv_image_depth = evo::evoMat2cvMat(evo_image_depth); //
            // std::cout << cv_image_depth.at<float>(100, 100) << std::endl;

            cv::cvtColor(cv_image_left, cv_image_left, CV_RGBA2BGR);
            cv::cvtColor(cv_image_right, cv_image_right, CV_RGBA2BGR);
            cv_image_depth = cv_image_depth.clone();//gray need no convert

            cv_image_left(cv::Rect(0, 0, 1280, 520)).copyTo(cv_image_left);
            cv_image_right(cv::Rect(0, 0, 1280, 520)).copyTo(cv_image_right);

            double now_1 = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            now_1 = now_1/1000;
            std::string left_img_path = data_dir_image_0 + to_string(counter) + ".png";
            std::string right_img_path = data_dir_image_1 + to_string(counter) + ".png";
            std::string depth_img_path = data_dir_image_depth + to_string(counter) + ".png"; //
            
            cv::imwrite(left_img_path, cv_image_left);
            cv::imwrite(right_img_path, cv_image_right);
            cv::imwrite(depth_img_path, cv_image_depth); //

            file_obj << std::setprecision(20) << now_1 << std::endl;

            cv::Mat concat;
            cv::vconcat(cv_image_left, cv_image_right, concat);
            cv::imshow("leadsense", cv_image_depth);
            cv::waitKey(1);
            
            counter ++; 
            // handleKey((char)cv::waitKey(1));

        }

        // Stop retrieve IMU data
		camera.stopRetrieveIMU();

        std::cout << std::endl << "run over" << std::endl;

        running = true;
        //evo Mat
    }
    cv::destroyAllWindows();
}
