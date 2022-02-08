#include "pibot_bringup/data_holder.h"
#include "pibot_bringup/CanOpen_driver.h"

#include <boost/assign/list_of.hpp>

CanOpenDriver* CanOpenDriver::instance = NULL;

CanOpenDriver::CanOpenDriver() : pn("~"), bdg(pn)
{
    bdg.init(&(Data_holder::get()->parameter));

    ROS_INFO("CanOpenDriver startup");

    if (init_can_device() < 0)
    {
        ROS_INFO("init_can_device failed!");
        return;
    }
    
    init_can_device();

    init_cmd_odom();
}

CanOpenDriver::~CanOpenDriver()
{
    if(instance != NULL)
        delete instance;
}

int CanOpenDriver::init_can_device()
{
    system("/home/nvidia/pibot_ros/scripts/setup_can.sh");

    struct sockaddr_can addr;
    struct ifreq ifr;
    //初始化CAN接口
    const char *ifname = "can0";
    if((can_device = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket \n "
               "try: sudo ip link set can0 up txqueuelen 1000 type can bitrate 1000000");
        return -1;
    }
    strcpy(ifr.ifr_name, ifname);
    ioctl(can_device, SIOCGIFINDEX, &ifr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    printf("%can_device at index %d\n", ifname, ifr.ifr_ifindex);
    if(bind(can_device, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }
}

void CanOpenDriver::init_cmd_odom()
{
    ROS_INFO_STREAM("subscribe cmd topic on [" << bdg.cmd_vel_topic << "]");
    cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &CanOpenDriver::cmd_vel_callback, this);

    ROS_INFO_STREAM("advertise odom topic on [" << bdg.odom_topic << "]");
    odom_pub = nh.advertise<nav_msgs::Odometry>(bdg.odom_topic, 50);

    //init odom_trans
    odom_trans.header.frame_id = bdg.odom_frame;  
    odom_trans.child_frame_id = bdg.base_frame;  

    odom_trans.transform.translation.z = 0;  

    //init odom
    odom.header.frame_id = bdg.odom_frame;  
    odom.pose.pose.position.z = 0.0;
    odom.child_frame_id = bdg.base_frame;  
    odom.twist.twist.linear.y = 0;  

    if (!bdg.publish_tf){
        odom.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ;
    
        odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ; 
    }
}

void CanOpenDriver::cmd_vel_callback(const geometry_msgs::Twist& msg)
{
    struct can_frame frame;

    int left_motor_speed = 0;
    int right_motor_speed = 0;
	//对读取的数据赋值
    float linear = msg.linear.x;
    float angular = msg.angular.z;
    left_motor_speed = (1.0/WHEEL_RADIUS*linear + ROBOT_RADIUS/WHEEL_RADIUS*angular)*60/PI*REDUCE_RATIO;
    right_motor_speed = (1.0/WHEEL_RADIUS*linear - ROBOT_RADIUS/WHEEL_RADIUS*angular)*60/PI*REDUCE_RATIO; 
    
    ROS_INFO_STREAM("[left_motor_speed is " << left_motor_speed << "] [right_motor_speed is " << right_motor_speed << "]");
    //参数下发
	//这里需要根据驱动器自行设置
    frame.can_id  = left_motor_NodeID;  //can的输出ip地址
    frame.can_dlc = 8;                  //数据位
    frame.data[0] = 0x23;               //具体数值，这个要根据不同的驱动器自己修改，不一定都一样
    frame.data[1] = 0xFF;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = left_motor_speed&0xff;
    frame.data[5] = (left_motor_speed&(0xff<<8))>>8;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;    
    write(can_device, &frame, sizeof(struct can_frame));
    
    usleep(200);     

    frame.can_id  = right_motor_NodeID;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = 0xFF;
    frame.data[2] = 0x60;
    frame.data[3] = 0x00;
    frame.data[4] = right_motor_speed&0xff;
    frame.data[5] = (right_motor_speed&(0xff<<8))>>8;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    write(can_device, &frame, sizeof(struct can_frame));
    
}

void CanOpenDriver::update_odom(){
    
}

void CanOpenDriver::update_speed(){

}

void CanOpenDriver::work_loop()
{
    ros::Rate loop(1000);
    while (ros::ok())
    {
        //boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();

        update_odom();

        update_speed();
		
        loop.sleep();

	    ros::spinOnce();
    }
}