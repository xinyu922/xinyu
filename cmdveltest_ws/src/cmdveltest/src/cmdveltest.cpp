#include <iostream>
#include <sstream>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <chrono>
#include <time.h>



using namespace std;

double now_start, now_end;

// void delay(unsigned int xms)  // xms代表需要延时的毫秒数
// {
//     unsigned int x,y;
//     for(x=xms;x>0;x--)
//         for(y=110;y>0;y--);
// }

void delay(int second)
{
    time_t start_time, cur_time;//定义时间变量
    time(&start_time); //获取time_t类型的开始时
    //获取time_t类型的时间，结束时间减去开始时间小于给定的时间则退出循环
    do {
    time(&cur_time);
    } while ((cur_time - start_time) < second);
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "cmdveltest");
    ros::NodeHandle cmdh;
    ros::Publisher cmdpub= cmdh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);;
    ros::Rate r(10);
    geometry_msgs::Twist twist;
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;

    while(ros::ok())
    {
        linear.x=0.5;
        linear.y=0;
        linear.z=0;//直行

        angular.x=0;
        angular.y=0;
        angular.z=0;//转圈

        twist.linear=linear;
        twist.angular=angular;
        cmdpub.publish(twist);
        delay(6);
        // sleep(1);

        linear.x=0;
        angular.z=0;
        twist.linear=linear;
        twist.angular=angular;
        cmdpub.publish(twist);
        delay(2);
        // sleep(3);


        linear.x=0;
        linear.y=0;
        linear.z=0;//直行
        
        angular.x=0;
        angular.y=0;
        angular.z=-1;//转圈

        twist.linear=linear;
        twist.angular=angular;
        cmdpub.publish(twist);

        // double now_start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // now_start = now_start / 1000;
        // cout << "now_start:" << std::setprecision(20) << now_start << endl;

        delay(1);
        // sleep(1);

        // double now_end = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // now_end = now_end / 1000;
        // cout << "now_end:" << std::setprecision(20) << now_end << endl;

        linear.x=0;
        angular.z=0;
        twist.linear=linear;
        twist.angular=angular;
        cmdpub.publish(twist);
        delay(2);

    }

    return 0;
}

