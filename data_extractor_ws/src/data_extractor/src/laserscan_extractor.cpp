//
// Created by wang on 18-11-26.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <sys/stat.h>
#include <fstream>
#include <iomanip>

std::string laserScan_dir = "sensor/LaserScan";
std::ofstream outfile;
// static int count = 0;

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
            if(a == -1)
            {
                mkdir(DirName,0755);
            }
            DirName[i] = '/';
        }
    }
    return 0;

}
void grablaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan_msg)
{
    outfile << std::setprecision(20) << laserScan_msg->header.stamp.toSec() << " "
    << laserScan_msg->angle_min << " " << laserScan_msg-> angle_max<< " " << laserScan_msg->angle_increment << laserScan_msg->time_increment<< " "
    << laserScan_msg->scan_time << " " << laserScan_msg->range_min << " " << laserScan_msg->range_max << " " << std::endl;
    // << laserScan_msg->ranges << " "<< laserScan_msg->intensities<< " " << std::endl;
     
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserScan_extractor");
    ros::NodeHandle nh;
    const char* LaserScandirName = laserScan_dir.c_str();
    createDirectoryEx(LaserScandirName);
    outfile.open(laserScan_dir+"/"+"laserScan.txt");

    ros::Subscriber laserscan_sub = nh.subscribe("/scan", 10, grablaserScan);
    ros::spin();
}

