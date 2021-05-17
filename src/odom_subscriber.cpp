#include "ros/ros.h"
#include "ITC/robot_odom.h"
#include "sstream"

void odom_callback(const ITC::robot_odom::ConstPtr& msg){
	float speed,position;
	speed=msg->Speed;
	position=msg->Position;
	ROS_INFO("speed=%.2f and position=%.2f",speed,position);
}
int main(int argc, char **argv){

	ros::init(argc,argv,"odom_subscriber");
	ros::NodeHandle n;
	
	ros::Subscriber sub=n.subscribe("odom_pub",1000,odom_callback);
	
	ros::spin();
	
	return 0;
	
}
