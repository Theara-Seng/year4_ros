#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
#include"eod_robot/EodMotorData.h"

using namespace std;
float max_linear_vel = 15.0;
float max_angular_vel = 15.0;

class TeleopJoy{
public:
  TeleopJoy();
private:
  void callBack(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Publisher motPub_;
  ros::Subscriber sub;
  int i_velLinear , i_velAngular;
};

TeleopJoy::TeleopJoy()
{   i_velLinear = 1;
	i_velAngular = 0; 
	n.param("axis_linear",i_velLinear,i_velLinear);
	n.param("axis_angular",i_velAngular,i_velAngular);
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callBack, this);
	motPub_ = n.advertise<eod_robot::EodMotorData>("eod_robot/motor_commands", 5);
}

void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
	/*geometry_msgs::Twist vel;
	vel.angular.z = max_angular_vel*joy->axes[3];
	vel.linear.x = max_linear_vel*joy->axes[1];
	pub.publish(vel);*/
	  eod_robot::EodMotorData motor;
  motor.flMotor = max_linear_vel*joy->axes[1] ;
  motor.blMotor = max_linear_vel*joy->axes[1] ;
  motor.frMotor = max_linear_vel*joy->axes[1] ;
  motor.brMotor = max_linear_vel*joy->axes[1] ;
  motor.header.stamp = ros::Time::now();
  motPub_.publish(motor);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "c8_teleop_joy");
	TeleopJoy teleop_robot;
	ros::spin();
}
