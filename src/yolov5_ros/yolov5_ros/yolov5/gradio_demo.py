import torch
import gradio as gr


title = "基于Gradio的yolov5演示项目"
desc = "调试iou,conf"
base_conf , base_iou = 0.25 , 0.45

def det_img(img, conf, iou):
    model.conf = conf
    model.iou = iou
    return model(img).render()[0]

model = torch.hub.load(
    # 指定默认 ws  目录
            "/home/wys/ros_ws/src/yolov5_ros/yolov5_ros/yolov5/",
            "custom",
            path = "../weights/best_10000+700_you.pt",
            source = 'local',
            force_reload=True
        )

# gr.Webcam()
gr.Interface(inputs= ["image", gr.Slider(minimum=0,maximum=1,value=base_conf),
                       gr.Slider(minimum=0,maximum=1,value=base_iou)], 
             outputs=["image"], 
             fn= det_img ,
             title=title,
             description=desc,
             live= True,
             examples=[["/home/wys/图片/4.png",base_conf,base_iou],
                       ["/home/wys/图片/5.png",base_conf,base_iou]
                       ]
             ).launch()
