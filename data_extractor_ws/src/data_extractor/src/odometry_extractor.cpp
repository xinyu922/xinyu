//
// Created by wang on 18-11-26.
//
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <sys/stat.h>
#include <iomanip>


std::string odom_dir = "sensor/Odometry";
std::ofstream outfile;

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

void grabOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    outfile << std::setprecision(20) << odom_msg->header.stamp.toSec() << " "
    << odom_msg->pose.pose.position.x << " " << odom_msg->pose.pose.position.y << " " << odom_msg->pose.pose.position.z << " "
    << odom_msg->pose.pose.orientation.x << " " << odom_msg->pose.pose.orientation.y << " " << odom_msg->pose.pose.orientation.z << " " << odom_msg->pose.pose.orientation.w << " "
    << odom_msg->twist.twist.linear.x << " " << odom_msg->twist.twist.linear.y << " " << odom_msg->twist.twist.linear.z << " " << std::endl;
//     << odom_msg->twist.twist.angular.x << " " << odom_msg->twist.twist.angular.y << " " << odom_msg->twist.twist.angular.z << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_extractor");
    ros::NodeHandle nh;
    const char* odomDirName = odom_dir.c_str();
    createDirectoryEx(odomDirName);
    outfile.open(odom_dir+"/"+"odometry.txt");

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, grabOdom);
    ros::spin();
}