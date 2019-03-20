#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>

using namespace std;
using namespace sl;

int main(int argc, char** argv)
{
    string data_dir = "/home/ros/Documents/zed_data/";
    string time_txt = "/home/ros/Documents/zed_data/times.txt";
    ofstream file_obj;
    file_obj.open(time_txt);
    InitParameters initParameters;
    initParameters.camera_fps = 10;
    initParameters.camera_resolution = RESOLUTION_HD720;
    initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
    initParameters.coordinate_system = COORDINATE_SYSTEM_LEFT_HANDED_Y_UP;
    initParameters.coordinate_units = UNIT_METER;

    Camera zed;
    // Open the camera
    ERROR_CODE err = zed.open(initParameters);
    if (err != SUCCESS)
    {
        cout << toString(err) << endl;
        zed.close();
        return 1;
    }

    // Print camera information
    printf("ZED Model           : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number   : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware        : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution: %dx%d\n", (int)zed.getResolution().width, (int)zed.getResolution().height);
    printf("ZED Camera FPS      : %d\n", (int)zed.getCameraFPS());

    // Create a Mat to store images
    sl::Mat left_image, right_image; //校正后的
    sl::Mat left_image_raw, right_image_raw; //未校正
    int counter = 0;
    char key = ' ';
    while (key != 'q')
    {
        if (zed.grab() == SUCCESS) {
            // Retrieve left image  校正后的
            zed.retrieveImage(left_image, VIEW_LEFT);       
            zed.retrieveImage(right_image, VIEW_RIGHT);

            // 未校正的图片
            zed.retrieveImage(left_image_raw, VIEW_LEFT_UNRECTIFIED);       
            zed.retrieveImage(right_image_raw, VIEW_RIGHT_UNRECTIFIED);

            cv::Mat cv_left = cv::Mat((int)left_image.getHeight(), (int)left_image.getWidth(), CV_8UC4, left_image.getPtr<sl::uchar1>(sl::MEM_CPU));        //校正后的
            cv::Mat cv_right = cv::Mat((int)right_image.getHeight(), (int)right_image.getWidth(), CV_8UC4, right_image.getPtr<sl::uchar1>(sl::MEM_CPU));

            cv::Mat cv_left_raw = cv::Mat((int)left_image_raw.getHeight(), (int)left_image_raw.getWidth(), CV_8UC4, left_image_raw.getPtr<sl::uchar1>(sl::MEM_CPU));
            cv::Mat cv_right_raw = cv::Mat((int)right_image_raw.getHeight(), (int)right_image_raw.getWidth(), CV_8UC4, right_image_raw.getPtr<sl::uchar1>(sl::MEM_CPU));

            std::string left_img_path = data_dir + "image_0/" + to_string(counter) + ".png";
            std::string right_img_path = data_dir + "image_1/" + to_string(counter) + ".png";

            std::string left_raw_img_path = data_dir + "image_0_raw/" + to_string(counter) + ".png";
            std::string right_raw_img_path = data_dir + "image_1_raw/" + to_string(counter) + ".png";

            cv_left(cv::Rect(0, 0, 1280, 520)).copyTo(cv_left);
            cv_right(cv::Rect(0, 0, 1280, 520)).copyTo(cv_right);

            cv_left_raw(cv::Rect(0, 0, 1280, 520)).copyTo(cv_left_raw);
            cv_right_raw(cv::Rect(0, 0, 1280, 520)).copyTo(cv_right_raw);

		    cv::imwrite(left_img_path, cv_left);
            cv::imwrite(right_img_path, cv_right);

		    cv::imwrite(left_raw_img_path, cv_left_raw);
            cv::imwrite(right_raw_img_path, cv_right_raw);

            double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            now = now / 1000;
//              std::cout << now << std::endl;
            file_obj<<std::setprecision(20) << now << std::endl;
            counter++;
		    
            
        }
    }

    // Exit
    zed.close();
    return EXIT_SUCCESS;
}
