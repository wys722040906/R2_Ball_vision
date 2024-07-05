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
// #include <VCOMCOMM.h>
// 我方红球
/** 已完成功能
 *  识别框中球计数
 *  识别目标球
*/
/**待完成
 *  球框任务调度
 *  视野获取
 *  5*4表格更新 5*3全局表格 视角动态更新 扫描一圈 搜集样本（实时扫描）
 *  框目标锁定
 *  边沿检测，特征提取，确定姿态
 *  面积采样确定框距
 * 
 * 找中心框范围-----------------------------！----------------
 * / 
/** 任务
 *  
 *  决策算法（返回一维数组 优先级从左到右 扫描一圈 确定目标）---(雷达备选)
 *  
 *  求反馈
 */ 

#define our_ball_name "red"
#define Low_Pass_Filter 0.6
#define left_basket 250
#define right_basket 450

typedef struct {
    int arr[5][4]={};
    float distance[5] ={};
}POSITION_LOCATION;
typedef struct {
	float weight;
    uint8_t id;
}Basket_Typedef;


static yolov5_ros_msgs::port_serial target_ball ;
static yolov5_ros_msgs::port_serial middle_ball ;

static int basket_rank[5] = {};
static int flag =1;
static int ball_count = 0;
static int last_basket = 0;
static int count = 0;
static int ballIn_basket = 0;
static int five_ball_count = 0;
static int less_five_ball_count =0;
static POSITION_LOCATION pos = {};


// 比较每个框的xmin
bool Basket_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b ){
    return a.xmin < b.xmin ;
}
// 比较球每个球ymax
bool Ball_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b ){
    return a.ymax > b.ymax ;
}
// 我方球中到(320,480)和上一球的最短距离
bool Our_ball_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b){

    return Low_Pass_Filter* ( ( pow((a.xmin + a.xmax)/2 - 320,2) + pow((a.ymax + a.ymin )/2 - 480 , 2)  )  )+ 
    (1- Low_Pass_Filter) * ( pow(((a.xmin + a.xmax )/2 - middle_ball.x ) , 2 ) + pow(((a.ymin+a.ymax)/2 - middle_ball.y),2) )<
    Low_Pass_Filter * (( pow((b.xmin + b.xmax)/2 - 320,2) + pow((b.ymax + b.ymin )/2 - 480 , 2)  ) )+
    (1- Low_Pass_Filter) * (pow(((b.xmin + b.xmax )/2 - middle_ball.x ) , 2 ) + pow(((b.ymin+b.ymax)/2 - middle_ball.y),2) )
    ;
}

bool WeightBall_Compare(const Basket_Typedef &a , const Basket_Typedef &b)
{
    return a.weight > b.weight;
}
// 输入5*4数组，得出排序
// 1 敌方  2 我方
//  红 1 蓝 2
//  接受到检测框信号， flag 至1， basket_rank清零 ， ball_count清零 ，last_basket清零 (flag 默认0,检测到置1) pos置零 five_ball_count = 0 less_five_ball_count = 0
int intobox(POSITION_LOCATION basket) 
{
	Basket_Typedef ball[5] = {{.weight = (float)500.0/basket.distance[0], .id = 1},{.weight = (float)500.0/basket.distance[1], .id = 2},{.weight = (float)500.0/basket.distance[2], .id = 3},{.weight = (float)500.0/basket.distance[3], .id = 4},{.weight = (float)500.0/basket.distance[4], .id = 5}};
	int i = 0;
	for(i = 0;i<5;i++)	
	{
		switch(basket.arr[i][0])
		{
			case 0: ball[i].weight += 1000; break;
			case 1: 
				ball[i].weight += 500; 
				switch(basket.arr[i][1])
				{
					case 1: ball[i].weight += 10; break;
					case 2: ball[i].weight += 30; break;
					default: break;
				}
				break;
			case 2: 
				ball[i].weight += 5000; 
				if(basket.arr[i][1] == basket.arr[i][2])
				{
					switch(basket.arr[i][1])
					{
						case 1: ball[i].weight += 3000; break;
						case 2: ball[i].weight += 1000; break;                                                  
						default: break;
					}
				}
				else if(basket.arr[i][1] != basket.arr[i][2])
				{
					ball[i].weight += 2000;
				}
				break;
			default: break; 
		}
	}
    int n = sizeof(ball) / sizeof(ball[0]);
    std::sort(ball, ball+n, WeightBall_Compare);
    // return ball[0].id;
    if(flag){
        for(int i=0; i< 5; i++){
            basket_rank[i] =ball[i].id;
        }        
        flag = 0;   
        ball_count = basket.arr[basket_rank[0]-1][0];
        last_basket = basket_rank[0];
        ROS_INFO("确定目标id:%d",last_basket);
        return last_basket;
    }    
    else if(ballIn_basket != ball_count) {
        five_ball_count ++;
        if(five_ball_count >= 10){
            last_basket = basket_rank[1];
            ROS_INFO("5框时目标变化%d",last_basket);
        }
        ROS_INFO("5框时目标没变%d",last_basket);
        return last_basket;
    }
    else {
        ROS_INFO("5框时目标没变%d",last_basket);
        return last_basket;
    }

}


POSITION_LOCATION update_basket(POSITION_LOCATION basket)
{
    static POSITION_LOCATION  basket_msg = {};
    for(int i = 0 ; i < 5; i++){
        if(basket_msg.arr[i][0] <= basket.arr[i][0]){
            basket_msg.arr[i][0] = basket.arr[i][0];
            basket_msg.arr[i][1] = basket.arr[i][1];
            basket_msg.arr[i][2] = basket.arr[i][2];
            basket_msg.arr[i][3] = basket.arr[i][3];
            basket_msg.distance[i] = basket.distance[i];     
        }
    }
    return basket_msg;
}

void message_callback(const yolov5_ros_msgs::BoundingBoxes & msg){
    POSITION_LOCATION position_location ;
    int16_t sum = msg.bounding_boxes[0].num ;
    int16_t i = 0 , j = 0  , m = 0 ;
    int16_t ballIn_basket = 0;
    int16_t basket_num = 0 , ball_num = 0 , our_ball_num = 0 ;
    yolov5_ros_msgs::BoundingBoxes basket_array ;
    yolov5_ros_msgs::BoundingBoxes ball_array ;
    yolov5_ros_msgs::port_serial basket ;
    yolov5_ros_msgs::port_serial target_basket ;
    yolov5_ros_msgs::port_serial ball ;
    yolov5_ros_msgs::BoundingBoxes our_ball_array = {};
// -------------------------球位置发布者对象
    static ros::NodeHandle nh;
    static ros::Publisher ball_pub = nh.advertise< yolov5_ros_msgs::port_serial>("target_ball", 10);
    static ros::Publisher basket_pub = nh.advertise<  yolov5_ros_msgs::port_serial>("target_basket", 10);


    ballIn_basket = 0;

    if(msg.bounding_boxes.size()==msg.bounding_boxes[0].num){
    // ROS_INFO("start manage image\n");
//--------------------------计总数 
    for(i = 0 , basket_num = 0 ;  i < msg.bounding_boxes[0].num  ; i++){
        if(msg.bounding_boxes[i].Class == "basket"){
            basket_array.bounding_boxes.push_back(msg.bounding_boxes[i]);
            basket_num ++ ;
        }
        if(msg.bounding_boxes[i].Class == our_ball_name ){
            our_ball_array.bounding_boxes.push_back(msg.bounding_boxes[i]);
            our_ball_num ++ ;
        }
        if((msg.bounding_boxes[i].Class == "blue" || 
        msg.bounding_boxes[i].Class =="red" ) && 
        msg.bounding_boxes[i].xmin < 320 && 
        msg.bounding_boxes[i].xmax > 320 )
            ballIn_basket++;
    }
    // ROS_INFO("己方框%d个",basket_num);
    // ROS_INFO("己方球%d个",our_ball_num);
    // ROS_INFO("屏幕中我方的球%d",our_ball_num);
// -------------------------球-----------------
    if (our_ball_num  == 0 || sum == 0 )  { middle_ball.x = 320 ; middle_ball.y = 480 ; middle_ball.distance = 0 ; middle_ball.id = 1 ;}
    else {
        std::sort(our_ball_array.bounding_boxes.begin() , our_ball_array.bounding_boxes.end() , Our_ball_Compare ) ;
        if(our_ball_num) {
        middle_ball.x = (int)round((our_ball_array.bounding_boxes[0].xmin + our_ball_array.bounding_boxes[0].xmax)/2) ;
        middle_ball.y = (int)round((our_ball_array.bounding_boxes[0].ymin + our_ball_array.bounding_boxes[0].ymax)/2) ;
        target_ball.x =  - (320 - round ( ( our_ball_array.bounding_boxes[0].xmax + our_ball_array.bounding_boxes[0].xmin )/2  ) );
        target_ball.y = 240 - round ( ( our_ball_array.bounding_boxes[0].ymax + our_ball_array.bounding_boxes[0].ymin )/2  ) ;
        target_ball.distance = round ( sqrt ( pow((our_ball_array.bounding_boxes[0].xmin + our_ball_array.bounding_boxes[0].xmax)/2 - 320,2) + 
        pow((our_ball_array.bounding_boxes[0].ymax + our_ball_array.bounding_boxes[0].ymin )/2 - 240 , 2 )  )); 
        target_ball.id = 1 ;  
        ball_pub.publish(target_ball) ;
        // ROS_INFO("目标球id%d\n",target_ball.id) ;
        // ROS_INFO("x坐标%d\n",target_ball.x) ;
        // ROS_INFO("y坐标%d\n",target_ball.y) ;
        // ROS_INFO("目标球距离(320,240)%d\n",target_ball.distance) ;
        }
        
    }

// ---------------------框------------------
    if(basket_num != 5){
/**
 * 清除球识别标志位
 * 策略确定后检测不到5个框-->监测目标框中球变化
 */         
        if(basket_num && !flag ){                   
                // if(ballIn_basket != ball_count){
                //    less_five_ball_count++;  
                //    if(less_five_ball_count>= 10){
                //     last_basket = basket_rank[1];
                //     ROS_INFO("三框时目标变化%d",last_basket);
                //    }          
                //    basket.id = last_basket;
                //    basket.x = 0;
                //    basket.y = 0;
                //    basket.distance = 0;
                //    basket_pub.publish(basket);
                //    ROS_INFO("三框时目标没变%d",last_basket);
                // }
                // else {
                //     basket.id = last_basket;
                //     basket.x = 0;
                //     basket.y =0;
                //     basket.distance = 0;
                //     ROS_INFO("三框时目标没变%d",last_basket);
                //     basket_pub.publish(basket);
                // }
            

        }
    }
    else {
        std::sort(basket_array.bounding_boxes.begin() , basket_array.bounding_boxes.end() , Basket_Compare ) ;
        // ROS_INFO("框数等于5个\n") ;
        
        for(i = 0 ;i < basket_num ; i++ ){
            for(j = 0 ; j < msg.bounding_boxes[0].num ; j++){  
               
                if(msg.bounding_boxes[j].Class != "basket" && 
                (msg.bounding_boxes[j].xmin+msg.bounding_boxes[j].xmax)/2 > basket_array.bounding_boxes[i].xmin &&
                (msg.bounding_boxes[j].xmin+msg.bounding_boxes[j].xmax)/2 < basket_array.bounding_boxes[i].xmax &&
                (msg.bounding_boxes[j].ymin+msg.bounding_boxes[j].ymax)/2 > basket_array.bounding_boxes[i].ymin - 40 &&
                (msg.bounding_boxes[j].ymin+msg.bounding_boxes[j].ymax)/2 < basket_array.bounding_boxes[i].ymax  ){
                    ball_array.bounding_boxes.push_back( msg.bounding_boxes[j] ) ;
                    ball_num++;
                }
                } 
                // ROS_INFO("%d框有%d个球", i + 1 , ball_num) ;
                
                std::sort(ball_array.bounding_boxes.begin() , ball_array.bounding_boxes.end() , Ball_Compare ) ;
                // for ( m = 0; m < ball_num ; m++ )
                // {
                //     ROS_INFO("%s\n" , ball_array.bounding_boxes[m].Class.c_str()) ; 
                // }
                position_location.distance[i] = sqrt ( pow ( ( ( basket_array.bounding_boxes[i].xmin + basket_array.bounding_boxes[i].xmax ) / 2 - 320 ) , 2 ) +
                     pow( (basket_array.bounding_boxes[i].ymax + basket_array.bounding_boxes[i].ymin ) / 2 - 480 , 2) );
                switch (ball_num)
                {
                case 0:
                    position_location.arr[i][0] = 0;
                    position_location.arr[i][1] = 0; 
                    position_location.arr[i][2] = 0;
                    position_location.arr[i][3] = 0;                    
                    break;
                case 1:
                    position_location.arr[i][0] = 1;
                    if(ball_array.bounding_boxes[0].Class == "red"){                   
                        position_location.arr[i][1] = 1;
                        position_location.arr[i][2] = 0;
                        position_location.arr[i][3] = 0;
                    }
                    else {
                        position_location.arr[i][1] = 2; 
                        position_location.arr[i][2] = 0;
                        position_location.arr[i][3] = 0;
                    }                    
                    break;
                case 2:
                    position_location.arr[i][0] = 2;
                    if(ball_array.bounding_boxes[0].Class == "red"){
                        if(ball_array.bounding_boxes[1].Class == "red"){
                                position_location.arr[i][1] = 1; 
                                position_location.arr[i][2] = 1;
                                position_location.arr[i][3] = 0;
                                }else {
                                position_location.arr[i][1] = 1; 
                                position_location.arr[i][2] = 2;
                                position_location.arr[i][3] = 0;
                                }
                    }else {
                            if(ball_array.bounding_boxes[1].Class == "red"){
                            position_location.arr[i][1] = 2; 
                            position_location.arr[i][2] = 1;
                            position_location.arr[i][3] = 0;
                            }else {
                            position_location.arr[i][1] = 2; 
                            position_location.arr[i][2] = 2;
                            position_location.arr[i][3] = 0;
                            }
                    }             
                    break;
                case 3:
                    position_location.arr[i][0] = 3;
                    if( ball_array.bounding_boxes[0].Class == "red"){
                        if(ball_array.bounding_boxes[1].Class == "red" ){
                            if(ball_array.bounding_boxes[2].Class == "red"){
                                position_location.arr[i][1] = 1; 
                                position_location.arr[i][2] = 1;
                                position_location.arr[i][3] = 1;
                            }
                            else{
                                position_location.arr[i][1] = 1; 
                                position_location.arr[i][2] = 1;
                                position_location.arr[i][3] = 2;
                            }
                        }
                        else {
                            if(ball_array.bounding_boxes[2].Class == "red"){
                                position_location.arr[i][1] = 1; 
                                position_location.arr[i][2] = 2;
                                position_location.arr[i][3] = 1;
                            }
                            else{
                                position_location.arr[i][1] = 1; 
                                position_location.arr[i][2] = 2;
                                position_location.arr[i][3] = 2;
                            }
                        }
                    }
                    else{
                        if(ball_array.bounding_boxes[1].Class == "red" ){
                            if(ball_array.bounding_boxes[2].Class == "red"){
                                position_location.arr[i][1] = 2; 
                                position_location.arr[i][2] = 1;
                                position_location.arr[i][3] = 1;
                            }
                            else{
                                position_location.arr[i][1] = 2; 
                                position_location.arr[i][2] = 1;
                                position_location.arr[i][3] = 2;
                            }
                        }
                        else {
                            if(ball_array.bounding_boxes[2].Class == "red"){
                                position_location.arr[i][1] = 2; 
                                position_location.arr[i][2] = 2;
                                position_location.arr[i][3] = 1;
                            }
                            else{
                                position_location.arr[i][1] = 2; 
                                position_location.arr[i][2] = 2;
                                position_location.arr[i][3] = 2;
                            }
                        }      

                    }
                    break;
                default:
                    ROS_INFO("框检测错误");
                    break;
                }
            //框遍历
            ball_num = 0 ;
            }
            // count ++;
            // if(count < 60){
            //   pos = update_basket(position_location);
            // }else{
                // count =0; 
                basket.id = 3;
                // intobox(pos) ;
                switch (basket.id)
                {
                case 1:
                    basket.x = ( basket_array.bounding_boxes[0].xmin + basket_array.bounding_boxes[0].xmin ) / 2 ; 
                    basket.y = ( basket_array.bounding_boxes[0].ymin + basket_array.bounding_boxes[0].ymax)  / 2 ;
                    basket.distance = position_location.distance[0];
                    break;
                case 2:
                    basket.x = ( basket_array.bounding_boxes[1].xmin + basket_array.bounding_boxes[1].xmin ) / 2 ; 
                    basket.y = ( basket_array.bounding_boxes[1].ymin + basket_array.bounding_boxes[1].ymax)  / 2 ;
                    basket.distance = position_location.distance[1];
                    break;
                case 3:
                    basket.x = ( basket_array.bounding_boxes[2].xmin + basket_array.bounding_boxes[2].xmin ) / 2 ; 
                    basket.y = ( basket_array.bounding_boxes[2].ymin + basket_array.bounding_boxes[2].ymax)  / 2 ;
                    basket.distance = position_location.distance[2];
                    break;
                case 4:
                    basket.x = ( basket_array.bounding_boxes[3].xmin + basket_array.bounding_boxes[3].xmin ) / 2 ; 
                    basket.y = ( basket_array.bounding_boxes[3].ymin + basket_array.bounding_boxes[3].ymax)  / 2 ;
                    basket.distance = position_location.distance[3];
                    break;
                case 5:
                    basket.x = ( basket_array.bounding_boxes[4].xmin + basket_array.bounding_boxes[4].xmin ) / 2 ; 
                    basket.y = ( basket_array.bounding_boxes[4].ymin + basket_array.bounding_boxes[4].ymax)  / 2 ;
                    basket.distance = position_location.distance[4];
                    break;
                default:
                    break;
                }
                // ROS_INFO("%d",basket.id);
                basket_pub.publish(basket) ;
                // ROS_INFO("等于5个框时目标框%d",last_basket);
                ROS_INFO("5个时目标框id%d\n",basket.id) ;
            // }


        }
    }
}



int main(int argc , char *argv[])
{
    setlocale(LC_ALL , "" );
    ros::init(argc , argv , "message_send_node");
    ros::NodeHandle sh;
    ros::Subscriber message_sub = sh.subscribe("/yolov5/BoundingBoxes" , 10 , message_callback);
    middle_ball.x = 320 ;
    middle_ball.y = 480 ;
    middle_ball.distance = 0 ;
    middle_ball.id = 0 ;
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}