#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopJoy
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  //int linear_, angular_;
  double joy_scale, left_scale_trigger,right_scale_trigger;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopJoy::TeleopJoy()//:
  //linear_(1),
  //angular_(2)
{

  nh_.param("joy_params/joy_scale", joy_scale);
  nh_.param("joy_params/left_scale_trigger", left_scale_trigger);
  nh_.param("joy_params/right_scale_trigger", right_scale_trigger);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  TeleopJoy teleop;

  ros::spin();
}