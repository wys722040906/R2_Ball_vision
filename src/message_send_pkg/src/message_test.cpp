#include <ros/ros.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/port_serial.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

void msg_CallBack(const yolov5_ros_msgs::port_serial::ConstPtr & msg){
    static int count = 0 ;
    ROS_INFO("id%d\n",count++);
    ROS_INFO("distance%d\n",msg->distance);
    ROS_INFO("x%d\n",msg->x);
    ROS_INFO("y%d\n",msg->y);
    ros::Duration(2.0).sleep() ;

}


int main(int argc , char *argv[])
{
    setlocale(LC_ALL , "" );
    ros::init(argc , argv , "message_test_node");
    ros::NodeHandle sh;
    ros::Subscriber sub = sh.subscribe("basket_id" , 10 , msg_CallBack);
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}