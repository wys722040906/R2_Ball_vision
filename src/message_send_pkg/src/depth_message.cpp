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

//   <node pkg="message_send_pkg" type="message_handle" name="message_handle" output="screen"/>


void image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    // ---------------异常处理机制
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        // 在这里可以对图像进行OpenCV操作
        cv::Mat image = cv_ptr->image;


        float depth = image.at<float>(240, 320) ;
        ROS_INFO("Depth is %.2f", depth);




    }
    // -----------------捕获到异常
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }


}

int main(int argc , char *argv[])
{
    setlocale(LC_ALL , "" );
    ros::init(argc , argv , "depth_message");
    ros::NodeHandle sh;
    image_transport::ImageTransport it(sh);
    image_transport::Subscriber image_sub = it.subscribe ("/camera/depth/image_raw" ,1, image_Callback) ;
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}