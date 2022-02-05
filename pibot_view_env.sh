#!/bin/bash

code_name=$(lsb_release -sc)

if [ "$code_name" = "trusty" ]; then
    ros_version="indigo"
elif [ "$code_name" = "xenial" ]; then
    ros_version="kinetic"
elif [ "$code_name" = "bionic" ]; then
    ros_version="melodic"
else
    echo -e "\033[1;31m PIBOT not support "$code_name"\033[0m"
    exit
fi 

LOCAL_IP=`ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | awk -F"/" '{print $1}'`

echo -e "\033[1;33mPIBOT_ENV_INITIALIZED:   "$PIBOT_ENV_INITIALIZED
echo -e "\033[1;34mSYS_DIST:                "$code_name
echo -e "\033[1;33mROS_DIST:                "$ros_version
echo -e "\033[1;34mLOCAL_IP:                "$LOCAL_IP
echo -e "\033[1;33mROS_MASTER_URI:          "$ROS_MASTER_URI
echo -e "\033[1;34mROS_IP:                  "$ROS_IP
echo -e "\033[1;33mROS_HOSTNAME:            "$ROS_HOSTNAME
echo -e "\033[1;34mPIBOT_MODEL:             "$PIBOT_MODEL
echo -e "\033[1;33mPIBOT_LIDAR:             "$PIBOT_LIDAR
echo -e "\033[1;34mPIBOT_BOARD:             "$PIBOT_BOARD
