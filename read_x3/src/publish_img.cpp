#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/ByteMultiArray.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

image_transport::Publisher image_pub_; 
void arrayCallback(const std_msgs::ByteMultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "publish_img");

	ros::NodeHandle n;	
    image_transport::ImageTransport it(n);
	ros::Subscriber sub3 = n.subscribe("image_array", 100, arrayCallback);
    image_pub_ = it.advertise("/x3/image_raw", 1);
	ros::spin();

	printf("\n");
	return 0;
}

void arrayCallback(const std_msgs::ByteMultiArray::ConstPtr& array)
{
     vector<signed char> vec = array->data;
     signed char* data = &vec[0];
     cv::Mat image = cv::Mat(480,640, CV_8UC3, data).clone();
     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
     image_pub_.publish(msg);
//     imshow("image", image);
//     waitKey(30);

	return;
}
