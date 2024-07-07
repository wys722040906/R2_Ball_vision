#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# 全局变量
image_count = 0 
save_frequency = 10  # 每10帧保存一张图片
save_path = '/home/wys/视频/image8'  # 保存图片的路径

# 回调函数，处理接收到的图像消息
def image_callback(msg):
    global image_count
    try:
        if image_count % save_frequency == 0:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            
            # 保存图片
            filename = os.path.join(save_path, '{}.jpg'.format(image_count // save_frequency))
            cv2.imwrite(filename, cv_image)
            print("Saved image:", filename)
            cv2.imshow("widow",cv_image)
        
        image_count += 1
        
    except Exception as e:
        print(e)

# 主函数
def main():
    rospy.init_node('image_saver', anonymous=True)

    # 订阅图像话题
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        # 按下 Ctrl+C 键，程序退出
        print("User exit")

if __name__ == '__main__':
    main()
