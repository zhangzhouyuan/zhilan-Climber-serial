/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动


*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
#include "zhilan_climber_serial/chassis.h"
#include "chassis_serial_communication.h"
using namespace std;
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

float D = 0.398000f ;    //两轮间距，单位是m
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度

string rec_buffer;  //串口数据接收变量

string port = "/dev/ttyUSB0";  //设置串口号
int baud = 115200;               //设置串口波特率



/************************************************************/
void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    float RightVel,LeftVel;
    int R_speed,L_speed,j;
    unsigned char upspeed[23];

    serial::Serial my_serial(port.c_str(), baud, serial::Timeout::simpleTimeout(1000)); //配置串口

    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s

    //将转换好的小车速度分量为左右轮速度
    LeftVel  = linear_temp - 0.5f*angular_temp*D ;
    RightVel = linear_temp + 0.5f*angular_temp*D ;
    //存入数据到要发布的左右轮速度消息
    L_speed = LeftVel*1000;   //放大１０００倍，mm/s
    R_speed = RightVel*1000;//放大１０００倍，mm/s

    set_speed(R_speed,L_speed);
    //set_speed(336,336);

    for(int i=0;i<23;i++){
        j=i;
        if (j>6) j++;
        upspeed[i] = setspeed.data[j];
    }

    //写入数据到串口
    my_serial.write(upspeed,23);

}

int main(int argc, char **argv)
{
    unsigned char um[11];
    int check,j;
    setlocale(LC_ALL, "");    //rosinfo输出中文

    ros::init(argc, argv, "chassis_serial_communication");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    ros::param::get("~serial_port", port);
	ros::param::get("~serial_baud", baud);

    ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题
    ros::Publisher  pub = n.advertise<zhilan_climber_serial::chassis>("VelSpeed",20);
    zhilan_climber_serial::chassis VelSPeed;

    serial::Serial my_serial(port.c_str(), baud, serial::Timeout::simpleTimeout(1000));//配置串口

    //停止上次开启的20ms上传模式
    stop_20update();
    for(int i=0;i<11;i++){
        j=i;
        if (j>6) j++;
        um[i] = updatemodel.data[j];
    }
    my_serial.write(&um[0],11);

    //发送查询产品类型
    Init_inquiar_product();
    my_serial.write(&inquiarProduct.data[0],12);

    //接收产品类型参数
    rec_buffer = my_serial.readline(25,"\n");
    const char *receive_data=rec_buffer.data();
    if(rec_buffer.length()==25){
        product_param(&rec_buffer[0]);
    }

    //开启20ms上传模式
    open_20update();
    for(int i=0;i<11;i++){
        j=i;
        if (j>6) j++;
        um[i] = updatemodel.data[j];
    }
    my_serial.write(&um[0],11);  //发送数据


    ros::Rate loop_rate(50);//设置周期休眠时间

    while(ros::ok())
    {
        //处理20ms上传发送的里程计数据
        rec_buffer =my_serial.readline(25,"\n");    //获取串口发送来的数据
        const char *receive_data=rec_buffer.data(); //保存串口发送来的数据
        if(rec_buffer.length()==25) {               //串口接收的数据长度正确就处理并发布里程计数据消息
            if(rec_buffer[23] == 0x0D && rec_buffer[24] == 0x0A) {
                if (rec_buffer[2] == 0x19) {
                    for (int i = 0; i < 25; i++) {
                        j = i;
                        if (j > 6) j++;
                        receivespeed.data[j] = rec_buffer[i];
                    }
                    for (int i = 0; i < 22; i++) {
                        check = check + (receivespeed.data[i] & 0x000000FF);
                    }

                    VelSPeed.FLVel = (float) receivespeed.st.F_left_speed_data.d / 1000.0;
                    VelSPeed.FRVel = (float) receivespeed.st.F_right_speed_data.d / 1000.0;
                    VelSPeed.BLVel = (float) receivespeed.st.B_left_speed_data.d / 1000.0;
                    VelSPeed.BRVel = (float) receivespeed.st.B_right_speed_data.d / 1000.0;
                    VelSPeed.Electric = receivespeed.st.Electric.d;
                    VelSPeed.state = receivespeed.st.State.d;
                    pub.publish(VelSPeed);  //发布里程计数据
                    ros::spinOnce();//周期执行
                    loop_rate.sleep();//周期休眠
                }
            }
        }
        ros::spinOnce();//周期执行
    }
    return 0;
}

/*******************************************************************
//Fun:Init_inquiar_product
//Descript:填充产品查询指令
//input：NONE
//return：NONE
*********************************************************************/
void Init_inquiar_product(){

    int check = 0;
    inquiarProduct.st.FrameHead.d = 0xDEED;
    inquiarProduct.st.FrameLen.d  = 0x0C;
    inquiarProduct.st.FrameType.d = 0x00;
    inquiarProduct.st.FrameCMD.d  = 0x0000;
    inquiarProduct.st.DATALEN.d   = 0x01;
    inquiarProduct.st.INFO.d      = 0x00;
    inquiarProduct.st.FrameTail.d = 0x0A0D;
    for(int i=0;i<8;i++){
        check = check + (inquiarProduct.data[i]&0x000000FF);
    }
    inquiarProduct.st.Framechecksum.d = check;
}


/*******************************************************************
//Fun:product_param
//Descript:接收产品类型参数
//input：NONE
//return：NONE
*********************************************************************/
void product_param(char rx[25]){
    int check,j;
    if(rx[23] == 0x0D && rx[24] == 0x0A){

        if(rx[2] == 0x19){
            for(int i=0;i<25;i++){
                j=i;
                if (j>6) j++;
                productparam.data[j] = rx[i];
            }
            for(int i=0;i<22;i++){
                check = check + (productparam.data[i]&0x000000FF);
            }
            if(check == productparam.st.Framechecksum.d){
                if(productparam.st.Receive_FrameType.d == 0 ){
                    ROS_INFO("查询产品类型失败");
                }
                else{
                    ROS_INFO("查询产品类型成功");
                }
                ROS_INFO("编码器线:%d",productparam.st.Encoder_line.d);
                ROS_INFO("电机最大转速:%d",productparam.st.Max_speed_motor.d);
                ROS_INFO("轮子X轴轴距:%d",productparam.st.X_Wheelbase.d);
                ROS_INFO("轮子Y轴轴距:%d",productparam.st.Y_Wheelbase.d);
                ROS_INFO("减速比:%d",productparam.st.Reduction_ratio.d);
                ROS_INFO("轮子直径:%d",productparam.st.Wheel_Diameter.d);
            }
        }
    }
}

/*******************************************************************
//Fun:open_20update
//Descript:开启主动上传模式
//input：NONE
//return：NONE
*********************************************************************/
void open_20update(){
    int check = 0;

    updatemodel.st.FrameHead.d = 0xDEED;
    updatemodel.st.FrameLen.d  = 0x0B;
    updatemodel.st.FrameType.d = 0x04;
    updatemodel.st.FrameCMD.d  = 0x8007;
    updatemodel.st.DATALEN.d   = 0x00;
    for(int i=0;i<8;i++){
        check = check + (updatemodel.data[i]&0x000000FF);
    }
    updatemodel.st.Framechecksum.d = check;
    updatemodel.st.FrameTail.d     = 0x0A0D;
    updatemodel.st.INFO.d =0x00;
}

/*******************************************************************
//Fun:stop_20update
//Descript:关闭主动上传模式
//input：NONE
//return：NONE
*********************************************************************/
void stop_20update(){
    updatemodel.st.FrameHead.d = 0xDEED;
    updatemodel.st.FrameLen.d  = 0x0B;
    updatemodel.st.FrameType.d = 0x04;
    updatemodel.st.FrameCMD.d  = 0x8007;
    updatemodel.st.DATALEN.d   = 0x00;
    updatemodel.st.Framechecksum.d = 0x0000;
    updatemodel.st.FrameTail.d     = 0x0A0D;
    updatemodel.st.INFO.d =0x00;
}

/*******************************************************************
//Fun:set_speed
//Descript:控制车辆移动速度
//input：左右轮速度
//return：NONE
*********************************************************************/
void set_speed(int R_speed,int L_speed){
    int check = 0;
    setspeed.st.FrameHead.d = 0xDEED;
    setspeed.st.FrameLen.d  = 0x17;
    setspeed.st.FrameType.d = 0x04;
    setspeed.st.FrameCMD.d  = 0x8002;
    setspeed.st.DATALEN.d   = 0x06;
    setspeed.st.INFO.d   = 0x00;
    setspeed.st.F_right_speed_control.d = R_speed;
    setspeed.st.F_left_speed_control.d  = L_speed;
    setspeed.st.B_right_speed_control.d = R_speed;
    setspeed.st.B_left_speed_control.d  = L_speed;
    setspeed.st.Stop.d                  = 0x0000;
    setspeed.st.State.d                 = 0x0000;
    for(int i=0;i<20;i++){
        check = check + (setspeed.data[i]&0x000000FF);
    }
    setspeed.st.Framechecksum.d = check;
    setspeed.st.FrameTail.d     = 0x0A0D;

}


