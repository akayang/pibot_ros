   ros ��ȡģ����Ϣ���� �汾��ROS Kinetic    ubuntu 16.04 64λ
   
1. �������뻷����

2. ���նˣ����������ռ䣬ִ����������
   mkdir my_test
   cd my_test
   source /opt/ros/kinetic/setup.sh
   mkdir -p ./src
   cd src
   catkin_init_workspace
   cd ..
   catkin_make

   ��ʱ�����build��devel�ļ��У�devel���м���setup.*sh��Ҫ���������

   source devel/setup.bash      //����ù����ռ�Ļ�������
   echo $ROS_PACKAGE_PATH       //��ȡ��������

3.�������ܰ�
   cd src

   catkin_create_pkg imu_publish std_msgs roscpp rospy

   ������imu_publish�ļ����е����ж������Ƹ��ǵ�my_test/src/imu_publish ��

   �л���my_test Ŀ¼��

   catkin_make

   ����ɹ�������my_test/devel/lib/imu_publish/�¿���һ��imu��ִ���ļ����������е� saber_cfg.json �ļ���imu����һ��


4. �����ն˽���my_test��������source devel/setup.bash ������������ roscore��

5. �����ն� ����my_testִ������ source devel/setup.bash
   ���������Ȩ�� sudo chmod 777 /dev/ttyUSB0 �������ļ���Ϊsaber_cfg.json ��

   cd devel/lib/imu_publish 

   ִ������ rosrun imu_publish imu    (imu_publish ��package�����֣� imu �ǿ�ִ���ļ���)

6. �����ն˽���catkin_ws2 ִ������ source devel/setup.bash
   
  ��һ�ַ�����ִ������ rostopic echo /Imu_data ���Կ���������topic��Imu_data.(�Ƽ�)
   
  �ڶ��ַ�����ִ������ rosrun imu_publish receive

7.����imuͬ��Ŀ¼�¿���imu.log����־�ļ��������н���4000��ֹͣ����main.c���޸ġ�
  