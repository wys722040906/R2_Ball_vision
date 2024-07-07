#include <ros/ros.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <opencv2/opencv.hpp>
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/port_serial.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace cv ;

cv::Scalar lower_purple_1(122, 0, 96); // HSV颜色空间下的低值
cv::Scalar upper_purple_1(146, 173, 255); // HSV颜色空间下的高值

cv::Scalar lower_purple_2(119, 56, 54); // HSV颜色空间下的低值
cv::Scalar upper_purple_2(140, 63, 255); // HSV颜色空间下的高值

cv::Scalar lower_purple_3(119, 56, 34); // HSV颜色空间下的低值
cv::Scalar upper_purple_3(140, 255, 255); // HSV颜色空间下的高值

cv::Scalar lower_purple_4(121, 17, 65); // HSV颜色空间下的低值
cv::Scalar upper_purple_4(155, 156, 255); // HSV颜色空间下的高值

cv::Scalar lower_purple_5(122, 0, 96); // HSV颜色空间下的低值
cv::Scalar upper_purple_5(146, 173, 255); // HSV颜色空间下的高值

cv::Scalar lower_purple_6(121, 64, 89); // HSV颜色空间下的低值
cv::Scalar upper_purple_6(145, 174, 255); // HSV颜色空间下的高值


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
        Mat img_HSV, result ,img_Blur; 
        Mat mask1, mask2, mask3, mask4, mask5, mask6;
        // 创建球发布者对象
        static ros::NodeHandle nh;
        static ros::Publisher video_pub = nh.advertise<sensor_msgs::Image>("video_topic", 10);
        //  COLOR_BGR2GRAY
        // cvtColor(image, image ,COLOR_RGB2BGR
        // );
        cv::GaussianBlur(image, img_Blur, cv::Size(5, 5), 0);
        // cvtColor(image, img_HSV ,COLOR_BGR2HSV);
        //  指定范围置255
        // inRange(img_Blur, lower_purple_1, upper_purple_1,mask1);
        // inRange(img_Blur, lower_purple_2, upper_purple_2,mask2);
        // inRange(img_HSV, lower_purple_3, upper_purple_3,mask3);
        // inRange(img_HSV, lower_purple_4, upper_purple_4,mask4);
        // inRange(img_HSV, lower_purple_5, upper_purple_5,mask5);
        // inRange(img_HSV, lower_purple_5, upper_purple_6,mask6);        

        // image.setTo(Scalar(0,0,0), mask1);
        // image.setTo(Scalar(0,0,0), mask2);
        // image.setTo(Scalar(0,0,0), mask3);
        // image.setTo(Scalar(0,0,0), mask4);
        // image.setTo(Scalar(0,0,0), mask5);
        // image.setTo(Scalar(0,0,0), mask6);

        // iamge 转 image_msg
        sensor_msgs::ImagePtr ros_image;
        try {
            ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        } 
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        imshow("video", image);
        
        //  发布image_msg
        video_pub.publish(ros_image);

        waitKey(30);
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
    ros::init(argc , argv , "/usb_cam/image_raw");
    ros::NodeHandle sh;
    image_transport::ImageTransport it(sh);
    image_transport::Subscriber image_sub = it.subscribe ("/usb_cam/image_raw" , 1 , image_Callback) ;
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}