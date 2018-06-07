#include <ros/ros.h>
//#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/dji_sdk_node.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
using namespace std;



vector<float> data;
unsigned char raw0[88], raw1[88]; 
float _left, _top, _right, _bottom;
vector<unsigned char> char_array;

ros::ServiceClient client;
ros::Publisher pub;
dji_sdk::SendMobileData srv;
//dji_sdk::SendDataToRemoteDevice srv;
int size_byte, size_float;
/*
void receiveCallback(const dji_sdk::MobileData::ConstPtr& array )
{
   data.clear();
   size_byte = array->data.size();
   size_float = size_byte/4;
   char_array = array->data;
   for(int i=0;i<size_float;i++){
      for(int j=0;j<4;j++){
         raw[i*4+j]=array->data[i*4+(3-j)];
      }
      //   printf("%d ", raw[i]);
   }
   ROS_INFO("received\n");


   srv.request.data.clear();
   for(int i=0;i<42;i++)
      srv.request.data.push_back(i);

      if(client.call(srv)){
       ROS_INFO("Message sent");
      }else{
       ROS_ERROR("Failed to call service");
      }
}
*/
void SendDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
   float tmp = 1;
   memcpy(raw0, &tmp, sizeof(float));
   tmp = -1;
   memcpy(raw1, &tmp, sizeof(float));

   vector<float> pose;
   for(std::vector<float>::const_iterator it = msg->data.begin(); it!=msg->data.end(); ++it){
      pose.push_back(*it);
   }
   for(int i=0; i<21; i++){
      tmp = pose[i];
      memcpy(raw0+4*(i+1), &tmp, sizeof(float));
      tmp = pose[21+i];
      memcpy(raw1+4*(i+1), &tmp, sizeof(float));

   }
   for(int i=0;i<22;i++){
       for(int j=0;j<2;j++){
          raw0[i*4+j]=raw0[i*4+(3-j)];
       }
   }
   for(int i=0;i<22;i++){
       for(int j=0;j<2;j++){
          raw1[i*4+j]=raw1[i*4+(3-j)];
       }
   }

   srv.request.data.clear();
   for(int i=0;i<88;i++)
        srv.request.data.push_back(raw0[i]);

   if(client.call(srv)){
       ROS_INFO("Message raw0 sent");
   }else{
       ROS_ERROR("Failed to call service");
   }


   srv.request.data.clear();
   for(int i=0;i<88;i++)
        srv.request.data.push_back(raw1[i]);

   if(client.call(srv)){
       ROS_INFO("Message raw1 sent");
   }else{
       ROS_ERROR("Failed to call service");
   }

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mobile_comm");

    ros::NodeHandle n;	
    //ros::Subscriber sub = n.subscribe("/djiros33/from_mobile_data", 100, receiveCallback);
    ros::Subscriber pose_sub = n.subscribe("/djiros33/norm3d", 100, SendDataCallback);
    client = n.serviceClient<dji_sdk::SendMobileData>("/djiros33/dji_sdk/send_data_to_mobile");
    //client = n.serviceClient<dji_sdk::SendDataToRemoteDevice>("dji_sdk/send_data_to_remote_device");

    ros::spin();

    return 0;
}
