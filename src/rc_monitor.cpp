#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <mav_manager/Vec4.h>
#include <memory>

using namespace std;

int points_added_counter;
ros::ServiceClient client;

float mult_vel = 0.01;
bool state_hover = true;
ros::Publisher vel_pub;
ros::Publisher hov_pub;
ros::ServiceClient client_vel;
ros::ServiceClient client_hov;

void rc_callback(const mavros_msgs::RCIn::ConstPtr &RC)
{
  float vel_y = 0.0;
  float vel_z = 0.0;

  //read channel for the thrust to give velocity on z
  if(RC->channels[2] > 1480 && RC->channels[2] < 1520)
    vel_z = 0;
  else if(RC->channels[2] >= 1520.0)
    vel_z = (RC->channels[2] - 1520.0) / 400 * mult_vel;//need to be scaled
  else if(RC->channels[2] <= 1480.0)
    vel_z = (RC->channels[2] - 1480.0) / 400 * mult_vel;//need to be scaled

  
  //read channel for the roll to give velocity on y
  if(RC->channels[0] > 1480 && RC->channels[0] < 1520)
    vel_y = 0;
  else if(RC->channels[0] >= 1520.0)
    vel_y = (RC->channels[0] - 1520.0) / 400 * mult_vel;//need to be scaled
  else if(RC->channels[0] <= 1480.0)
    vel_y = (RC->channels[0] - 1480.0) / 400 * mult_vel;//need to be scaled
    
  //call the velocity tracker
  /*geometry_msgs::Twist msg_vel;
  msg_vel.linear.x = 0.0f;
  msg_vel.linear.y = vel_y;
  msg_vel.linear.z = vel_z;
  msg_vel.angular.z = 0.0f;
  std_msgs::Empty empyt_msg;
  if(RC->channels[4] <= 1500){
    vel_pub.publish(msg_vel);
    state_hover = false;
  }
  else if(!state_hover){
    //switch to position control and hover
      msg_vel.linear.x = 0.0f;
      msg_vel.linear.y = 0.0f;
      msg_vel.linear.z = 0.0f;
      msg_vel.angular.z = 0.0f;
      vel_pub.publish(msg_vel);
      hov_pub.publish(empyt_msg);
    state_hover = true;
  }*/
  
  mav_manager::Vec4 srv_vel;
    srv_vel.request.goal[0]  = 0.0f;
  srv_vel.request.goal[1]  = vel_y;
  srv_vel.request.goal[2]  = vel_z;
  srv_vel.request.goal[3]  = 0.0f;
  
    mav_manager::Vec4 srv_hov;
    srv_hov.request.goal[0]  = 0.0f;
  srv_hov.request.goal[1]  = 0.0f;
  srv_hov.request.goal[2]  = 0.0f;
  srv_hov.request.goal[3]  = 0.0f;
  
  if(RC->channels[4] <= 1500){


  if (client_vel.call(srv_vel))
  {
    ROS_INFO("Sum: %ld", (bool)srv_vel.response.success);
    state_hover = false;
  }
  else
    ROS_ERROR("Failed to call service velocity");
  }
  else if(!state_hover){
    
    srv_vel.request.goal[0]  = 0.0f;
    srv_vel.request.goal[1]  = 0.0f;
    srv_vel.request.goal[2]  = 0.0f;
    srv_vel.request.goal[3]  = 0.0f;
    
  if (client_vel.call(srv_vel))
    ROS_INFO("Sum: %ld", (long int)srv_vel.response.success);
  else
    ROS_ERROR("Failed to call service velocity before hover");
   
  if (client_hov.call(srv_hov))
    ROS_INFO("Sum: %ld", (bool)srv_hov.response.success);
  else
    ROS_ERROR("Failed to call service hover");
    state_hover = true;
  }
  
  
}


int main(int argc, char **argv)
{

    ros::init(argc,argv,"rc_monitor");

    ros::NodeHandle n("~");
    
      //read the external velocity option
      n.param("mult_vel", mult_vel, 0.5f);
      //send the commands
      //vel_pub = n.advertise<geometry_msgs::Twist>("setDesVelInWorldFrame", 10);
      //hov_pub = n.advertise<std_msgs::Empty>("hover", 10);

    client_vel = n.serviceClient<mav_manager::Vec4>("setDesVelInWorldFrame");
    client_hov = n.serviceClient<mav_manager::Vec4>("hover");
    ros::Subscriber rc_sub_ = n.subscribe("rc/in", 10, &rc_callback,
                               ros::TransportHints().tcpNoDelay());
    
    ros::spin();

    return 0;
}
