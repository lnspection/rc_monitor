#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <memory>

using namespace std;

int points_added_counter;
ros::ServiceClient client;

float mult_vel = 0.01;
bool state_hover = false;
ros::Publisher vel_pub;
ros::Publisher hov_pub;

void rc_callback(const mavros_msgs::RCIn::ConstPtr &RC)
{
  float vel_y = 0.0;
  float vel_z = 0.0;

  //read channel for the thrust to give velocity on z
  if(RC->channels[2] > 1480 && RC->channels[2] < 1520)
    vel_z = 0;
  else if(RC->channels[2] >= 1520)
    vel_z = (RC->channels[2] - 1520) / 400 * mult_vel;//need to be scaled
  else if(RC->channels[2] <= 1480)
    vel_z = (RC->channels[2] - 1480) / 400 * mult_vel;//need to be scaled
  
  //read channel for the roll to give velocity on y
  if(RC->channels[2] > 1480 && RC->channels[2] < 1520)
    vel_y = 0;
  else if(RC->channels[2] >= 1520)
    vel_y = (RC->channels[2] - 1520) / 400 * mult_vel;//need to be scaled
  else if(RC->channels[2] <= 1480)
    vel_y = (RC->channels[2] - 1480) / 400 * mult_vel;//need to be scaled
    
  //call the velocity tracker
  geometry_msgs::Twist msg_vel;
  msg_vel.linear.x = 0.0f;
  msg_vel.linear.y = vel_y;
  msg_vel.linear.z = vel_z;
  msg_vel.angular.z = 0.0f;
  std_msgs::Empty empyt_msg;
  if(RC->channels[6] == 1){
    vel_pub.publish(msg_vel);
    state_hover = false;
  }
  else if(!state_hover){
    //switch to position control and hover
    hov_pub.publish(empyt_msg);
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
      vel_pub = n.advertise<geometry_msgs::Twist>("setDesVelInWorldFrame", 10);
      hov_pub = n.advertise<std_msgs::Empty>("hover", 10);

    
    ros::Subscriber rc_sub_ = n.subscribe("rc/in", 10, &rc_callback,
                               ros::TransportHints().tcpNoDelay());
    
    ros::spin();

    return 0;
}
