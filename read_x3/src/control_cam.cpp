#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
// publisher handles
ros::Publisher goal_pub;
void publish_goal()
{ 
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "base_link";
  msg.pose.position.x = 0; 
  msg.pose.position.y = 0; 
  msg.pose.position.z = 0;   
  msg.pose.orientation.w = 0; 
  msg.pose.orientation.x = 0; 
  msg.pose.orientation.y = 0; 
  msg.pose.orientation.z = 30;  
  goal_pub.publish(msg);
}
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "move_cam");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/djiros/gimbal_ctrl", 1);
    while (ros::ok()) 
    {
      publish_goal();
      ros::spinOnce();
      loop_rate.sleep();
    }
}
