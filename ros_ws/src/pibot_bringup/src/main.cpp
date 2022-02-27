
#define SERIAL_DRIVER
//#define CANOPEN_DRIVER

#include <ros/ros.h>
#if defined SERIAL_DRIVER
    #include "pibot_bringup/base_driver.h"
#else if defined CANOPEN_DRIVER
    #include "pibot_bringup/CanOpen_driver.h"
#endif


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pibot_driver");

#if defined SERIAL_DRIVER    
    BaseDriver::Instance()->work_loop();
#else if defined CANOPEN_DRIVER
    CanOpenDriver::Instance()->work_loop();
#endif

    ros::spin();

    return 0;
}