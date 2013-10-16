//      carmen_teleop_joy.cpp
//      
//      Based upon http://www.ros.org/wiki/joy/Tutorials/WritingTeleopNode

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>            // sensor_msgs
#include <geometry_msgs/Twist.h>		// cmd_vel


class TeleopCARMEN
{
public:
  TeleopCARMEN();
  bool isAutoSend();
  void sendVels();
  bool ok();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int trans_vel_axis;
  int rot_vel_axis;  
  double trans_scale;
  double rot_scale;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  bool auto_send;
  geometry_msgs::Twist cmd_vel;
  
};


TeleopCARMEN::TeleopCARMEN():
  trans_vel_axis(1),
  rot_vel_axis(0),
  trans_scale(1),
  rot_scale(1),
  auto_send(true)
{

  nh_.param("trans_vel_axis", trans_vel_axis, trans_vel_axis);
  nh_.param("rot_vel_axis", rot_vel_axis, rot_vel_axis);
  nh_.param("rot_scale", rot_scale, rot_scale);
  nh_.param("trans_scale", trans_scale, trans_scale);
  nh_.param("auto_send", auto_send, auto_send);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopCARMEN::joyCallback, this);

}

bool TeleopCARMEN::isAutoSend() {
	return auto_send;
}

bool TeleopCARMEN::ok() {
	return nh_.ok();
}

void TeleopCARMEN::sendVels(){
    vel_pub_.publish(cmd_vel);
}

void TeleopCARMEN::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
   
  cmd_vel.linear.x = rot_scale*joy->axes[trans_vel_axis];
  cmd_vel.angular.z = trans_scale*joy->axes[rot_vel_axis];
  
  //por si acaso ...
  cmd_vel.linear.y= 0.0;
  cmd_vel.linear.z= 0.0;
  cmd_vel.angular.x=0.0;
  cmd_vel.angular.y=0.0;
  
  ROS_INFO("[teleop_carmen] received joy  (%3.3f,%3.3f)",joy->axes[trans_vel_axis],joy->axes[rot_vel_axis]);
  ROS_INFO("[teleop_carmen] casted to vel (%3.3f m/s , %3.3f rad/s )",cmd_vel.linear.x,cmd_vel.angular.z);
  
  sendVels();
}


int main(int argc, char** argv)
{
  double refresh_rate = 50;
  ros::init(argc, argv, "teleop_carmen");
  TeleopCARMEN teleop_carmen;
  
  ROS_INFO("[teleop_carmen] running");
  ros::Rate r(refresh_rate);

  if (teleop_carmen.isAutoSend()){	
      while(teleop_carmen.ok()){
        teleop_carmen.sendVels();
        ros::spinOnce();    
        r.sleep();
      }
  } else {
      ros::spin();
  }

  ROS_INFO("[teleop_carmen] shut down");
}
