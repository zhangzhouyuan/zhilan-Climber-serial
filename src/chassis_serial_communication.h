#ifndef __CHASSIS_SERIAL_COMMUNICATION_H
#define __CHASSIS_SERIAL_COMMUNICATION_H

union int2char //union的作用为实现char数组和float之间的转换
{
    short int d;
    char data[2];
}FrameHead,FrameCMD,Framechecksum,FrameTail,
 F_Left_speed_control,R_Right_speed_control,
 B_Left_speed_control,B_Right_speed_control,
 F_right_speed_data,F_left_speed_data,
 B_right_speed_data,B_left_speed_data,
 Encoder_line,Max_speed_motor,X_Wheelbase,Y_Wheelbase,Reduction_ratio,Wheel_Diameter,
 Electric,Receive_FrameType,State,Stop;

union char2char
{
    char data[1];
    char d;
}FrameLen,FrameType,DATALEN,INFO;

struct Inquiar_product{
    int2char  FrameHead;
    char2char FrameLen;
    char2char FrameType;
    int2char  FrameCMD;
    char2char DATALEN;
    char2char INFO;
    int2char  Framechecksum;
    int2char  FrameTail;
};

union struct2char12
{
    struct Inquiar_product st;
    unsigned char data[12];
}inquiarProduct;

struct Product_param{
    int2char  FrameHead;
    char2char FrameLen;
    char2char FrameType;
    int2char  FrameCMD;
    char2char DATALEN;
    char2char INFO;
    int2char  Receive_FrameType;
    int2char  Encoder_line;
    int2char  Max_speed_motor;
    int2char  X_Wheelbase;
    int2char  Y_Wheelbase;
    int2char  Reduction_ratio;
    int2char  Wheel_Diameter;
    int2char  Framechecksum;
    int2char  FrameTail;
};

union struct2char26
{
    struct Product_param st;
    char data[26];
}productparam;

struct Open_update_model{
    int2char  FrameHead;
    char2char FrameLen;
    char2char FrameType;
    int2char  FrameCMD;
    char2char DATALEN;
    char2char INFO;
    int2char  Framechecksum;
    int2char  FrameTail;
};

union struct2char12_2
{
    struct Open_update_model st;
    unsigned char data[12];
}updatemodel;

struct Receive_4speed{
    int2char  FrameHead;
    char2char FrameLen;
    char2char FrameType;
    int2char  FrameCMD;
    char2char DATALEN;
    char2char INFO;
    int2char  F_right_speed_data;
    int2char  F_left_speed_data;
    int2char  B_right_speed_data;
    int2char  B_left_speed_data;
    int2char  Electric;
    int2char  State;
    int2char  Framechecksum;
    int2char  FrameTail;
};

union struct2char26_2
{
    struct Receive_4speed st;
    char data[26];
}receivespeed;

struct Control_4speed{
    int2char  FrameHead;
    char2char FrameLen;
    char2char FrameType;
    int2char  FrameCMD;
    char2char DATALEN;
    char2char INFO;
    int2char  F_right_speed_control;
    int2char  F_left_speed_control;
    int2char  B_right_speed_control;
    int2char  B_left_speed_control;
    int2char  Stop;
    int2char  State;
    int2char  Framechecksum;
    int2char  FrameTail;
};

union struct2char24
{
    struct Control_4speed st;
    unsigned char data[24];
}setspeed;

void Init_inquiar_product();
void product_param(char*);
void open_20update();
void set_speed(int,int);
void stop_20update();


#endif //__CHASSIS_SERIAL_COMMUNICATION_H
