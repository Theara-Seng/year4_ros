#include "ros/ros.h"
#include "ITC/robot_odom.h"
#include "sstream"


int main(int argc,char **argv){

	ros::init(argc,argv,"odom_publisher");
	ros::NodeHandle n;
	ros::Publisher pub=n.advertise<ITC::robot_odom>("odom_pub",1000);
	ros::Rate loop_rate(10);
	
	ITC::robot_odom msg;
	msg.Speed=0.0;
	msg.Position=0.0;
	
	while (ros::ok()){
		pub.publish(msg);
		msg.Speed+=2.0;
		msg.Position=msg.Speed/10.0;
		ROS_INFO("speed=%.2f and position=%.2f",msg.Speed,msg.Position);
		ros::spinOnce();
		loop_rate.sleep();
	
	}

	return 0;

}
