#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

"""

import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes, port_serial
from cmath import sqrt

Our_Ball = lambda : "red"
Low_Pass_Filter = lambda : 0.7
Min_Area = lambda :  20


area = lambda xmin, xmax, ymin, ymax  : (xmax - xmin) * (ymax - ymin) 
class Low_pass_filter:

    target_position = {'x' : 320, 'y' : 480 } 
    middle_position = {'xmin' : 0, 'ymin' : 0 , 'xmax': 0 , 'ymax' :0} 

    def __init__(self) -> None:
        self.first_ball_flag = 0 
     
    """
    某点到 (320,240) 和上一个target
    """
    def lpf_distance(self ,xmin,xmax,ymin,ymax,) :
        return  \
                int (  sqrt( Low_Pass_Filter()*(pow(abs((int (xmin) + int (xmax) )/2 - 320) , 2) + 
                        pow(abs((int (ymin) +int (ymax) )/2 - 480) , 2) ) + \
                        (1-Low_Pass_Filter())*(pow(abs((int(xmin)+int (xmax))/2 - self.target_position['x']) , 2) + 
                        pow(abs((int (ymin)+int (ymax))/2 - self.target_position['y']) , 2))
                ).real )
    
    """
    (320,480)
    """
    def distance(self ,xmin,xmax,ymin,ymax,) :
        return sqrt( pow(((xmin+xmax)/2 - self.target_position['x']), 2) + pow((ymin + ymax)/2 - self.target_position['y'] , 2) ).real
    
    """
    初始化
    """
    def position_init(self) -> None:
        self.target_position['x'] = 320
        self.target_position['y'] = 480
        self.middle_position['xmin'] = 0
        self.middle_position['ymin'] = 0
        self.middle_position['xmax'] = 0
        self.middle_position['ymax'] = 0
               
# class Calman_filter():

#     def __init__(self) -> None:
#         self.conf = 10
#         self.purple_last = list()
#         self.purple_now = list()
#         pass
#     def get_purpleball(self, boxs):
#         self.purple_last = [ x for x in boxs if x[-1] == "purple" ] 

#     def distance(self,):

        
    
    
#     def compare_purpleball(self, boxs):
#         for x in boxs:
#            for y in self.purple_last :
#                if  
            

    
    

low_pass_filter = Low_pass_filter()

class Yolo_Dect:
    def __init__(self):
        # load parameters
        yolov5_path = rospy.get_param('/yolov5_path', '')
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param(
            '~image_topic', '/camera/color/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = rospy.get_param('~conf', '0.5')
        iou = rospy.get_param('~iou', '0.45')

        # load local repository(YoloV5:v6.0)
        self.model = torch.hub.load(yolov5_path, 'custom',
                                    path=weight_path, source='local')

        # which device will be used
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()

        self.model.conf = conf
        self.model.iou = iou
        self.color_image = Image()
        self.depth_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        # output publishers
        self.position_pub = rospy.Publisher(
            pub_topic,  BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher(
            '/yolov5/detection_image',  Image, queue_size=1)
    
        # if no image messages
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model(self.color_image)
        # xmin    ymin    xmax   ymax  confidence  class    name
        boxs = results.pandas().xyxy[0].values

        self.dectshow(self.color_image, boxs, image.height, image.width)

        cv2.waitKey(3)

    def dectshow(self, org_img, boxs, height, width):
        img = org_img.copy()

        low_pass_filter.first_ball_flag = 1  #检测我方第一球标志位

        i , sum ,count = 0, 0, 0 
        for i in boxs:
            count += 1
            if i[-1] == Our_Ball() :
                sum += 1 
        # cv2.putText(img, str(sum),
        #             (int(320), int(240)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)  
        ymin = 0
        ymax = 0
        # 30  75  154 208 276 330  409 460 548 596 
        """
        画框
        """
        # basket_list = list() 
        # basket_detect = [[27,83,ymin,ymax,0],[153,205,ymin,ymax,0],[270,324,ymin,ymax,0],[403,450,ymin,ymax,0],[535,590,ymin,ymax,0]]
        # for i in boxs:
        #     if i[-1] == "basket":
        #         j += 1
        #         ymin += i[1]
        #         ymax += i[3]
        #         basket_list.append([int (i[0]),int (i[2]) ,int(i[1]),int(i[3])])
        # cv2.putText(img, str(j),
        #             (int(320), int(240)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)    
        # ymin /= 3
        # ymin = int (ymin)
        # ymax /= 3
        # ymax = int (ymax)
        # if j >= 3 :
        #     for i in boxs:
        #         if i[-1] == "basket":
        #             x = (i[0] + i[2])/2
        #             if x >= 28 and x <= 75:
        #                 basket_detect[0][4] = 1
        #             elif x >= 150 and x <= 210:
        #                 basket_detect[1][4] = 1
        #             elif x >= 270 and x <= 330:
        #                 basket_detect[1][4] = 1
        #             elif x >= 410 and x <= 460:
        #                 basket_detect[1][4] = 1
        #             elif x >= 540 and x <= 600:
        #                 basket_detect[1][4] = 1
                    
        #     for i in basket_detect:
        #         if i[4] == 0 :
        #             basket_list.append(i)
        #             cv2.rectangle(img,( int(i[0]), int(i[2])),
        #                 (int(i[1]), int(i[3])), (0,0,255), 2)
                  
# --------------敌方球干扰
        """
        数框中的球
        """
        basket_list = [x  for x in boxs  if x[-1] == "basket"] 
        ball_list = [x for x in boxs if x[-1] == "red" or "blue"]
        for i in basket_list :
            basket_ball_list = [x for x in ball_list if (x[0] + x[2])/2 > i[0] and 
                                (x[0] + x[2])/2 < i[2] and
                                (x[1] + x[3])/2 > i[1] - 40 and 
                                (x[1] + x[3])/2 < i[3] and
                                (x[3] - x[1])/(x[2] - x[0]) < 1.3 and
                                area(int(x[0]),int(x[2]),int(x[1]),int(x[3])
                                     ) > Min_Area()
                                ]
            cv2.putText(img,str(len(basket_ball_list)) , (int(i[0]),int(i[1] - 20 ) ) , 
                        cv2.FONT_HERSHEY_SCRIPT_SIMPLEX , 1 ,(255 ,255 ,255), 2 ,cv2.LINE_AA)               
            basket_ball_list.clear()
            cv2.putText(img, str(1),
                     (int(320), int(240)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA) 

        for box in boxs:
            boundingBox = BoundingBox()
            boundingBox.probability =np.float64(box[4])
            boundingBox.xmin = np.int64(box[0])
            boundingBox.ymin = np.int64(box[1])
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])
            boundingBox.num = np.int16(count)
            boundingBox.Class = box[-1]

            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color

            cv2.rectangle(img, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)

            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10
            # 打标签
            cv2.putText(img, box[-1],
                    (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

            # 相似度
            cv2.putText(img, str(round(box[4],2)) ,
                (int((box[0] + box[2])/2) , int((box[1])+45)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)  
            """
                低通滤波   #可存入一个list , 并map , 在 filter
            """
            if box[-1] == Our_Ball() and low_pass_filter.first_ball_flag:
                    low_pass_filter.middle_position['xmin'] = int (box[0] )
                    low_pass_filter.middle_position['ymin'] = int (box[1] )
                    low_pass_filter.middle_position['xmax'] = int (box[2] )
                    low_pass_filter.middle_position['ymax'] = int (box[3] )
                    low_pass_filter.first_ball_flag = 0
            if box[-1] == Our_Ball() and \
                low_pass_filter.first_ball_flag == 0 and \
                low_pass_filter.lpf_distance(int (box[0]) , int (box[2]), int (box[1]),int (box[3])) < \
                    low_pass_filter.lpf_distance(low_pass_filter.middle_position['xmin'],low_pass_filter.middle_position['xmax'],\
                                                 low_pass_filter.middle_position['ymin'],low_pass_filter.middle_position['ymax']) :
                    low_pass_filter.middle_position['xmin'] = int (box[0] )
                    low_pass_filter.middle_position['ymin'] = int (box[1] )
                    low_pass_filter.middle_position['xmax'] = int (box[2] )
                    low_pass_filter.middle_position['ymax'] = int (box[3] )
            
            #  打印距离
            if box[-1] == Our_Ball():
                    cv2.putText(img, str(round(low_pass_filter.distance(box[0],box[2],box[1],box[3]))) ,
                    (int((box[0] + box[2])/2) , int((box[1])+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)            
            
            self.boundingBoxes.bounding_boxes.append(boundingBox)
            self.position_pub.publish(self.boundingBoxes)
        if sum :
            low_pass_filter.target_position['x'] = int (( low_pass_filter.middle_position['xmin'] + low_pass_filter.middle_position['xmax'] )/2 )
            low_pass_filter.target_position['y'] = int (( low_pass_filter.middle_position['ymin'] + low_pass_filter.middle_position['ymax'] )/2 )  
            cv2.putText(img, "target",
                    (low_pass_filter.target_position['x'] , low_pass_filter.target_position['y'] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1, cv2.LINE_AA)
            sum = 0
        if sum == 0 : 
            low_pass_filter.position_init()
        cv2.line(img, (320, 0), (320, 480), (0, 0, 0), 1)
        self.publish_image(img, height, width)
        cv2.imshow('YOLOv5', img)



    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


if __name__ == "__main__":
    rospy.init_node('yolov5_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()