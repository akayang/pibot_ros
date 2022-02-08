#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// create the TeleopXbox class and define the joyCallback function that will take a joy msg
class TeleopXbox
{
public:
  TeleopXbox();
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  geometry_msgs::Twist twist; 

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  int linear_, angular_;   // used to define which axes of the joystick will control our turtle
  double l_scale_, a_scale_;
};

TeleopXbox::TeleopXbox(): linear_(1), angular_(2)
{
  //  initialize some parameters
  nh_.param("axis_linear", linear_, linear_);  
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  // create a publisher that will advertise on the command_velocity topic of the turtle
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // subscribe to the joystick topic for the input to drive the turtle
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopXbox::joyCallback, this);
}


void TeleopXbox::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the turtle
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
}

int main(int argc, char** argv)
{
  // initialize our ROS node, create a teleop_xbox, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "teleop_xbox");
  TeleopXbox teleop_xbox;
  ros::Rate loop_rate(100);


  while(ros::ok()) {
    teleop_xbox.vel_pub_.publish(teleop_xbox.twist); 

    ros::spinOnce();
    loop_rate.sleep();
  }
}