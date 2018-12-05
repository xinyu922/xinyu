//
// Created by wang on 18-11-26.
//
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <sys/stat.h>
#include <fstream>
#include <iomanip>

std::string imu_dir = "sensor/imu";
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

void grabImu(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    outfile << std::setprecision(20) << imu_msg->header.stamp.toSec() << " "
    << imu_msg->orientation.x << " " << imu_msg->orientation.y << " " << imu_msg->orientation.z << imu_msg->orientation.w << " "
    << imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << " "
    << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_extractor");
    ros::NodeHandle nh;
    const char* imuDirName = imu_dir.c_str();
    createDirectoryEx(imuDirName);
    outfile.open(imu_dir+"/"+"imu.txt");

    ros::Subscriber imu_sub = nh.subscribe("/imu0", 10, grabImu);
    ros::spin();
}