import sys
import torch
from PySide6.QtWidgets import QMainWindow, QApplication, QFileDialog
from image_ui import  Ui_MainWindow 
from PySide6.QtGui import QPixmap, QImage
import cv2
from PySide6.QtCore import QTimer
# array 传进来 创建Qiamge
def convert2QImage(img):
    height , width , channel = img.shape
    return QImage(img, width, height, width*channel, QImage.Format_RGB888)
    

class MainWidow(QMainWindow, Ui_MainWindow):
    def __init__(self) :
        super(MainWidow, self).__init__()
        self.setupUi(self)  #超类初始化
        # model 加载
        self.model = torch.hub.load(
            "/home/wys/ros_ws/src/yolov5_ros/yolov5_ros/yolov5/",
            "custom",
            path = "../weights/best_10000+700_you.pt",
            source = 'local',
            force_reload=True
        )
        self.model.iou = 0.43
        self.model.conf = 0.8
        self.timer = QTimer()
        self.timer.setInterval(100)  #   ms间隔 
        self.video = None
        self.bind_slots() #槽和信号绑定

    def open_iamge(self):
        print("点击检测图片")
        self.timer.stop()
        file_path = QFileDialog.getOpenFileName(self,dir="/home/wys/CLionProjects/Opencv_4h/image", filter="*.jpg ; *.png")
        print(file_path)
        if file_path[0]:
            file_path = file_path[0]
            qimage = self.image_pred(file_path)
            self.input.setPixmap(QPixmap(file_path))  #图像显示
            self.output.setPixmap(QPixmap.fromImage(qimage))

    def image_pred(self, file_path):
        result = self.model(file_path)
        image = result.render()[0]
        return convert2QImage(image)

    def open_video(self):
        print("点击检测视频")
        file_path = QFileDialog.getOpenFileName(self,dir="/home/wys/视频", filter="*.mp4")
        if file_path[0]:
            file_path = file_path[0]
            self.video = cv2.VideoCapture(file_path)
            self.timer.start()
           
    def video_pred(self):
        ret, frame = self.video.read()
        if not ret:
            self.timer.stop()
        else:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.input.setPixmap(QPixmap.fromImage(convert2QImage(frame)))  #图像显示
            result = self.model(frame)
            image = result.render()[0]
            self.output.setPixmap(QPixmap.fromImage(convert2QImage(image)))                


    def bind_slots(self):
        self.det_video.clicked.connect(self.open_video)
        self.det_img.clicked.connect(self.open_iamge)
        self.timer.timeout.connect(self.video_pred)

    
if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWidow()
    window.show()
    app.exec()
