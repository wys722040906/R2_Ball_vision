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

// 定义颜色范围
cv::Scalar lowerColor(122, 121, 115); // HSV颜色空间下的低值
cv::Scalar upperColor(149, 173, 255); // HSV颜色空间下的高值

void image_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    // ---------------异常处理机制
    try
    {
        // 将ROS图像消息转换为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // 在这里可以对图像进行OpenCV操作
        cv::Mat image = cv_ptr->image;
        
        // 创建球发布者对象
        static ros::NodeHandle nh;
        static ros::Publisher message_pub = nh.advertise< std_msgs::String >("ballcolor", 10);

        // // 定义ROI
        cv::Rect roi(270 , 200 , 100 , 80 );
        cv::Mat roiImage=cv_ptr->image(roi);
        image = image(roi);
        // // // 定义HSV颜色空间
        cv::Mat hsvImage;
        cv::cvtColor(roiImage, hsvImage, cv::COLOR_BGR2HSV);
        // //  // 在颜色范围内创建掩码
        cv::Mat mask;
        cv::inRange(hsvImage, lowerColor, upperColor, mask);
        
        // // // 统计掩码中白色像素的数量
        int whitePixelCount = countNonZero(mask);
        // int whitePixelCount = 10 ;
        // 如果白色像素数量大于阈值，则说明这个区域包含指定颜色
        if (whitePixelCount < 50) {
        ROS_INFO("0");
        } else {
        ROS_INFO("1");
        }
    
        cv::waitKey(10);
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
    ros::init(argc , argv , "message_handle_node");
    ros::NodeHandle sh;
    image_transport::ImageTransport it(sh);
    image_transport::Subscriber image_sub = it.subscribe ("/usb_cam/image_raw" , 1 , image_Callback) ;
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}