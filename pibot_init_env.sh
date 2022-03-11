#!/bin/bash

sudo ln -sf ~/pibot_ros/pibot_init_env.sh /usr/bin/pibot_init_env
sudo ln -sf ~/pibot_ros/pibot_view_env.sh /usr/bin/pibot_view_env
sudo ln -sf ~/pibot_ros/pibot_install_ros.sh /usr/bin/pibot_install_ros

if ! [ $PIBOT_ENV_INITIALIZED ]; then
    echo "export PIBOT_ENV_INITIALIZED=1" >> ~/.bashrc
    echo "source ~/.pibotrc" >> ~/.bashrc

    #rules
    echo -e "\033[1;32m setup pibot modules"
    echo " "
    sudo cp rules/pibot.rules  /etc/udev/rules.d
    sudo cp rules/rplidar.rules  /etc/udev/rules.d
    sudo cp rules/ydlidar.rules  /etc/udev/rules.d
    sudo cp rules/orbbec.rules  /etc/udev/rules.d
    echo " "
    echo "Restarting udev"
    echo ""

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    #sudo service udev reload
    #sudo service udev restart
fi

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


content="#source ros

if [ ! -f /opt/ros/${ros_version}/setup.bash ]; then 
    echo \"please run cd ~/pibot_ros && ./pibot_install_ros.sh to install ros sdk\"
else
    source /opt/ros/${ros_version}/setup.bash
fi
"
echo "${content}" > ~/.pibotrc

#LOCAL_IP=`ifconfig eth0|grep "inet addr:"|awk -F":" '{print $2}'|awk '{print $1}'`
#LOCAL_IP=`ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | awk -F"/" '{print $1}'`

#if [ ! ${LOCAL_IP} ]; then
#    echo "please check network"
#    exit
#fi

LOCAL_IP=`ip addr | grep 'dynamic' -A0 | tail -n1 | awk '{print $2}' | awk -F"/" '{print $1}'`
echo "LOCAL_IP=\`ip addr | grep 'dynamic' -A0 | tail -n1 | awk '{print \$2}' | awk -F"/" '{print \$1}'\`" >> ~/.pibotrc

if [ ! ${LOCAL_IP} ]; then
    echo -e "\033[1;31m please check network\033[0m"
    exit
fi

echo -e "\033[1;34mplease specify pibot model\033[1;32m(0:apollo,1:apolloX,2:zeus,3:hera,4:hades,5:hadesX,6:xbot,other for user defined):\033[1;33m"
read -p "" PIBOT_MODEL_INPUT

if [ "$PIBOT_MODEL_INPUT" = "0" ]; then
    PIBOT_MODEL='apollo'
elif [ "$PIBOT_MODEL_INPUT" = "1" ]; then
    PIBOT_MODEL='apolloX'
elif [ "$PIBOT_MODEL_INPUT" = "2" ]; then
    PIBOT_MODEL='zeus'
elif [ "$PIBOT_MODEL_INPUT" = "3" ]; then
    PIBOT_MODEL='hera'
elif [ "$PIBOT_MODEL_INPUT" = "4" ]; then
    PIBOT_MODEL='hades'
elif [ "$PIBOT_MODEL_INPUT" = "5" ]; then
    PIBOT_MODEL='hadesX'
elif [ "$PIBOT_MODEL_INPUT" = "6" ]; then
    PIBOT_MODEL='xbot'
else
    PIBOT_MODEL=$PIBOT_MODEL_INPUT 
fi

echo -e "\033[1;34mplease specify pibot driver board type\033[1;32m(0:arduino(mega2560),1:stm32f103,2:stm32f407,other for user defined):\033[1;33m"
read -p "" PIBOT_DIRVER_BOARD_INPUT

if [ "$PIBOT_DIRVER_BOARD_INPUT" = "0" ]; then
    PIBOT_DRIVER_BAUD=115200
    PIBOT_BOARD='arduino'
elif [ "$PIBOT_DIRVER_BOARD_INPUT" = "1" ]; then
    PIBOT_DRIVER_BAUD=115200
    PIBOT_BOARD='stm32f1'
elif [ "$PIBOT_DIRVER_BOARD_INPUT" = "2" ]; then
    PIBOT_DRIVER_BAUD=921600
    PIBOT_BOARD='stm32f4'
else
    PIBOT_DRIVER_BAUD=115200
    PIBOT_BOARD=$PIBOT_DIRVER_BOARD_INPUT
fi

python ros_ws/src/pibot_bringup/scripts/set_baud.py $PIBOT_DRIVER_BAUD

echo -e "\033[1;34mplease specify your pibot lidar\033[1;32m(0:rplidar(a1,a2),1:rplidar(a3),2:eai(x4),3:eai(g4),4:xtion,5:astra,6:kinectV1,7:kinectV2,8:rplidar(s1),9:ltme,other for user defined):\033[1;33m"
read -p "" PIBOT_LIDAR_INPUT

if [ "$PIBOT_LIDAR_INPUT" = "0" ]; then
    PIBOT_LIDAR='rplidar'
elif [ "$PIBOT_LIDAR_INPUT" = "1" ]; then
    PIBOT_LIDAR='rplidar-a3'
elif [ "$PIBOT_LIDAR_INPUT" = "2" ]; then
    PIBOT_LIDAR='eai-x4'
elif [ "$PIBOT_LIDAR_INPUT" = "3" ]; then
    PIBOT_LIDAR='eai-g4'
elif [ "$PIBOT_LIDAR_INPUT" = "4" ]; then
    PIBOT_LIDAR='xtion'
elif [ "$PIBOT_LIDAR_INPUT" = "5" ]; then
    PIBOT_LIDAR='astra'
elif [ "$PIBOT_LIDAR_INPUT" = "6" ]; then
    PIBOT_LIDAR='kinectV1'
elif [ "$PIBOT_LIDAR_INPUT" = "7" ]; then
    PIBOT_LIDAR='kinectV2'
elif [ "$PIBOT_LIDAR_INPUT" = "8" ]; then
    PIBOT_LIDAR='rplidar-s1'
elif [ "$PIBOT_LIDAR_INPUT" = "9" ]; then
    PIBOT_LIDAR='ltme'
else
    PIBOT_LIDAR=$PIBOT_LIDAR_INPUT
fi

if [ "$PIBOT_LIDAR" = "kinectV2" ]; then
    ln -s $PWD/third_party/iai_kinect2 $PWD/ros_ws/src/
else
    if [ -f $PWD/ros_ws/src/iai_kinect2 ]; then
        rm $PWD/ros_ws/src/iai_kinect2
    fi
fi

echo "export ROS_IP=\`echo \$LOCAL_IP\`" >> ~/.pibotrc
echo "export ROS_HOSTNAME=\`echo \$LOCAL_IP\`" >> ~/.pibotrc
echo "export PIBOT_MODEL=${PIBOT_MODEL}" >> ~/.pibotrc
echo "export PIBOT_LIDAR=${PIBOT_LIDAR}" >> ~/.pibotrc
echo "export PIBOT_BOARD=${PIBOT_BOARD}" >> ~/.pibotrc

echo -e "\033[1;34mplease specify the current machine(ip:$LOCAL_IP) type\033[1;32m(0:pibot board,other:control PC):\033[1;33m" 
read -p "" PIBOT_MACHINE_VALUE
if [ "$PIBOT_MACHINE_VALUE" = "0" ]; then
    ROS_MASTER_IP_STR="\`echo \$LOCAL_IP\`"
    ROS_MASTER_IP=`echo $LOCAL_IP`
else
    echo -e "\033[1;34mplase specify the pibot board ip for commnication:\033[1;33m" 
    read -p "" PIBOT_ONBOARD_MACHINE_IP
    ROS_MASTER_IP_STR=`echo $PIBOT_ONBOARD_MACHINE_IP`
    ROS_MASTER_IP=`echo $PIBOT_ONBOARD_MACHINE_IP`
fi

echo "export ROS_MASTER_URI=`echo http://${ROS_MASTER_IP_STR}:11311`" >> ~/.pibotrc

echo -e "\033[1;35m*****************************************************************"
echo "model:        " $PIBOT_MODEL 
echo "lidar:        " $PIBOT_LIDAR  
echo "local_ip:     " ${LOCAL_IP} 
echo "onboard_ip:   " ${ROS_MASTER_IP}
echo ""
echo -e "please execute \033[1;36;4msource ~/.bashrc\033[1;35m to make the configure effective\033[0m"
echo -e "\033[1;35m*****************************************************************\033[0m"

#echo "source ~/pibot_ros/ros_ws/devel/setup.bash" >> ~/.pibotrc 
content="#source pibot

if [ ! -f ~/pibot_ros/ros_ws/devel/setup.bash ]; then 
    echo \"please run cd ~/pibot_ros/ros_ws && catkin_make to compile pibot sdk\"
else
    source ~/pibot_ros/ros_ws/devel/setup.bash
fi
"
echo "${content}" >> ~/.pibotrc

#alias
echo "alias pibot_bringup='roslaunch pibot_bringup bringup.launch'" >> ~/.pibotrc 
echo "alias pibot_bringup_with_imu='roslaunch pibot_bringup bringup_with_imu.launch'" >> ~/.pibotrc 
echo "alias pibot_lidar='roslaunch pibot_bringup ${PIBOT_LIDAR}.launch'" >> ~/.pibotrc 
echo "alias pibot_base='roslaunch pibot_bringup robot.launch'" >> ~/.pibotrc 
echo "alias pibot_base_with_imu='roslaunch pibot_bringup robot_with_imu.launch'" >> ~/.pibotrc 
echo "alias pibot_control='roslaunch pibot keyboard_teleop.launch'" >> ~/.pibotrc 
echo "alias pibot_configure='rosrun rqt_reconfigure rqt_reconfigure'" >> ~/.pibotrc 
echo "alias pibot_simulator='roslaunch pibot_simulator nav.launch'" >> ~/.pibotrc 

echo "alias pibot_gmapping='roslaunch pibot_navigation gmapping.launch'" >> ~/.pibotrc 
echo "alias pibot_gmapping_with_imu='roslaunch pibot_navigation gmapping_with_imu.launch'" >> ~/.pibotrc 
echo "alias pibot_save_map='roslaunch pibot_navigation save_map.launch'" >> ~/.pibotrc 

echo "alias pibot_naviagtion='roslaunch pibot_navigation nav.launch'" >> ~/.pibotrc 
echo "alias pibot_naviagtion_with_imu='roslaunch pibot_navigation nav_with_imu.launch'" >> ~/.pibotrc 
echo "alias pibot_view='roslaunch pibot_navigation view_nav.launch'" >> ~/.pibotrc 

echo "alias pibot_cartographer='roslaunch pibot_navigation cartographer.launch'" >> ~/.pibotrc 
echo "alias pibot_view_cartographer='roslaunch pibot_navigation view_cartographer.launch'" >> ~/.pibotrc 

echo "alias pibot_hector_mapping='roslaunch pibot_navigation hector_mapping.launch'" >> ~/.pibotrc 
echo "alias pibot_hector_mapping_without_imu='roslaunch pibot_navigation hector_mapping_without_odom.launch'" >> ~/.pibotrc 

echo "alias pibot_karto_slam='roslaunch pibot_navigation karto_slam.launch'" >> ~/.pibotrc 