#ifndef _PID_H_
#define _PID_H_

#include "stdint.h"


typedef struct
{
    float err;
    float err_last;
    float expect;
    float feedback;
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_max;
    float out;
    float out_max;
}_PID;

typedef struct
{
	/* 飞轮平衡pid */
	_PID rol_encoder;		//编码器速度环	
	_PID rol_angle;			//姿态外环
	_PID rol_gyro;      //角速度环    

	_PID yaw_encoder;		//编码器速度环	
	_PID yaw_angle;			//姿态外环
	_PID yaw_gyro;      //角速度环   	
}_PID_TYPE;

extern _PID_TYPE pidType;
extern const float  controller_parameter[6][5];

float pid_controller(_PID *controller);
void pid_init(void);
void clear_integral(_PID *controller);



#endif

