#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <mav_manager/manager.h>


// Typedefs
typedef Eigen::Vector4d Vec4;

class MAV_Subscribers
{
  public:

  // Let's make an MAV
  MAVManager mav_;

  void motors_cb(const std_msgs::Bool &msg)
  {
    mav_.set_motors(msg.data);
  }
  void takeoff_cb(const std_msgs::Empty &msg)
  {
    ROS_INFO("Trying to takeoff");
    if (!mav_.takeoff())
    {
      ROS_ERROR("Takeoff failed");
    }
  }
  void goTo_cb(const geometry_msgs::Pose &msg)
  {
    if (!mav_.goTo(msg.position.x, msg.position.y, msg.position.z, tf::getYaw(msg.orientation)))
    {
      ROS_ERROR("GoTo failed");
    }
  }
  void setDesVelInWorldFrame_cb(const geometry_msgs::Twist &msg)
  {
    mav_.setDesVelInWorldFrame(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
  }
  void setDesVelInBodyFrame_cb(const geometry_msgs::Twist &msg)
  {
    mav_.setDesVelInBodyFrame(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z);
  }
  void hover_cb(const std_msgs::Empty &msg)
  {
    if (!mav_.hover())
    {
      ROS_ERROR("Hover failed");
    }
  }
  void ehover_cb(const std_msgs::Empty &msg)
  {
    if (!mav_.ehover())
    {
      ROS_ERROR("Ehover failed");
    }
  }
  void estop_cb(const std_msgs::Empty &msg)
  {
    mav_.estop();
  }
};