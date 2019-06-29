#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
 
std::string turtle_name;
 
void poseCallback(const turtlesim::PoseConstPtr& msg){
	

 }
 
 int main(int argc, char** argv){
   ros::init(argc, argv, "my_tf2_broadcaster");
   ros::NodeHandle private_node("~");
static tf2_ros::TransformBroadcaster br;

	while(ros::ok())
	{
geometry_msgs::TransformStamped transformStamped;
   		transformStamped.header.stamp = ros::Time::now();
   		transformStamped.header.frame_id = "zedshow";
   		transformStamped.child_frame_id = "left_camera_optical_frame";
   		transformStamped.transform.translation.x = 0;
   		transformStamped.transform.translation.y = 0;
	   	transformStamped.transform.translation.z = 0;
	  	tf2::Quaternion q;
	   	q.setRPY(-1.57, 0, -1.57);
	   transformStamped.transform.rotation.x = q.x();
	   transformStamped.transform.rotation.y = q.y();
	   transformStamped.transform.rotation.z = q.z();
	   transformStamped.transform.rotation.w = q.w();
	 
   br.sendTransform(transformStamped);
	}

   return 0;
 };
