#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include "base_driver_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include "pibot_bringup/transport.h"
#include "pibot_bringup/dataframe.h"
#include <pibot_msgs/RawImu.h>

class CanOpenDriver
{
private:
    CanOpenDriver();

public:
    static CanOpenDriver* Instance()
    {
      if (instance == NULL)
        instance = new CanOpenDriver();

      return instance;
    }
    ~CanOpenDriver();
    void work_loop();
private:
    void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd);
    int init_can_device();
    void init_cmd_odom();
    void update_odom();
    void update_speed();
public:
  BaseDriverConfig& getBaseDriverConfig(){
    return bdg;
  }
  ros::NodeHandle* getNodeHandle(){
    return &nh;
  }
  ros::NodeHandle* getPrivateNodeHandle(){
    return &pn;
  }
private:
    static const float ROBOT_RADIUS = 0.44;     //  m
    static const float WHEEL_RADIUS = 0.15 ;    //  m
    static const float REDUCE_RATIO = 40;
    static const float PI = 3.1415926;

    static const int left_motor_NodeID = 0x601;
    static const int right_motor_NodeID = 0x602;

    int can_device;

    static CanOpenDriver* instance;
    BaseDriverConfig bdg;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;    
    ros::NodeHandle nh;
    ros::NodeHandle pn;
};
