//
// Created by wang on 18-11-26.
//

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <fstream>
#include <sys/stat.h>
#include <string>
#include <iomanip>

std::string image_0 = "sensor/stereo/image/image_0/data";
std::string image_1 = "sensor/stereo/image/image_1/data";
std::string depth = "sensor/stereo/depth/data";
std::string time_file = "sensor/stereo/image/time.txt";
std::ofstream outfile;
static int count = 0;

int createDirectoryEx(const char *sPathName )
{
    char DirName[256];
    strcpy(DirName,sPathName);
    int i,len = strlen(DirName);
    if(DirName[len-1]!='/')
        strcat(DirName,"/");
    len = strlen(DirName);
    for(i=1;i<len;i++)
    {
        if(DirName[i]=='/')
        {
            DirName[i] = 0;
            int a = access(DirName, F_OK);
            if(a ==-1)
            {
                mkdir(DirName,0755);
            }
            DirName[i] = '/';
        }
    }
    return 0;
}

void grabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight)
{
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
//    std::cout << image_1+"/"+std::to_string(count)+".png" << std::endl;
    cv::imwrite(image_0+"/"+std::to_string(count)+".png", cv_ptrLeft->image);
    cv::imwrite(image_1+"/"+std::to_string(count)+".png", cv_ptrRight->image);
    outfile << std::setprecision(20) << cv_ptrLeft->header.stamp.toSec() << std::endl;
    count++;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_extractor");
    ros::NodeHandle nh;
    const char* image0Name = image_0.c_str();
    const char* image1Name = image_1.c_str();
    const char* depthName = depth.c_str();
    createDirectoryEx(image0Name);
    createDirectoryEx(image1Name);
    createDirectoryEx(depthName);
    outfile.open(time_file);

    // message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam0/image_raw", 10);
    // message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam1/image_raw", 10);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/rgb/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/depth/image_raw", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(grabStereo);

    ros::spin();
}