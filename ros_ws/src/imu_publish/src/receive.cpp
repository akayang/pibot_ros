#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <time.h>
#include <string.h>

using namespace std;

void chatterCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
	//ROS_INFO("	time: %ld",msg->header.stamp);
	ROS_INFO(" linear_acc: 	\t%11.4f, %11.4f, %11.4f ***",msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

	ROS_INFO(" 	gyro:	\t%11.4f, %11.4f, %11.4f ***",msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

	ROS_INFO("orientation:	\t%11.4f, %11.4f, %11.4f, %11.4f ***",msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

	ROS_INFO("    ");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "receive");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("Imu_data", 20, chatterCallback);
 
	ros::spin();
 
	return 0;
}
