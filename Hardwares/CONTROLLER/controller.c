#include "controller.h"
#include "imu.h"
#include "pid.h"
#include "encoder.h"
#include "mpu6050.h"
#include "imath.h"
#include "math.h"
#include "tim.h"
#include <stdlib.h>
#include "servo.h"
#include "bluetooth.h"
#include "calibration.h"
_OUT_Motor motor1 = {0};
_OUT_Motor motor2 = {0};
_OUT_Motor motor3 = {0};

_PID_CONTROL vel = {4.0f, 0.02f, 5.4f};       			/* 速度环参数 */
_PID_CONTROL bal = {466.6f, 0.0f , 22.8f};  				/* 直立环参数 */

_OUT_CONTROL ctr = {0};

/**
  * @brief   控制前后行走
  * @param   cmd 遥控指令
  * @retval  value 返回控制速度：速度为固定值100
  */
#define vSPEED 70000.0F
float go_straight(float cmd)
{
  static float value = 0;
  if(cmd == 0){
		if(value < vSPEED){
			value += 100.0F;		
		}
  }
  else if(cmd == 5){
		if(value > -vSPEED){
			value -= 100.0F;		
		}
  }
  else{
		value = 0;
  }  
  return value;
}


#define ADJUST          // 需要微调时打开，默认不打开
  #if defined(ADJUST)
  #define DIR_BASE 150    // 方向基值
#define DIR_VARIATE -30 // 方向微调变量值
#endif

/**
  * @brief   控制左右转向
  * @param   cmd 遥控指令
  * @retval  value 返回控制速度：速度为固定值150
  */
float turn_round(float cmd, float front_back_cmd)
{
	float value = 0;
	if( 0 == cmd ){       // 摇杆方向向左打
#if defined(ADJUST)    
    if(front_back_cmd == 0){       // 摇杆方向向前打（蓝牙模块方向）
      value = DIR_BASE + DIR_VARIATE;			    
    }
		else if(front_back_cmd == 5){  //  摇杆方向向后打
      value = DIR_BASE - DIR_VARIATE;
    }	
#else 
    value = 150;
#endif		
	}
	else if( 5 == cmd ){  // 摇杆方向向右打
#if defined(ADJUST)    
    if(front_back_cmd == 0){       // 摇杆方向向前打（蓝牙模块方向）
      value = -(DIR_BASE - DIR_VARIATE);			    
    }
		else if(front_back_cmd == 5){  //  摇杆方向向后打
      value = -(DIR_BASE + DIR_VARIATE);
    }		
#else 
    value = -150;
#endif	
	}
	else{
		value = 0;
	}
	return value;
}

/**
  * @brief   外环速度控制器
  */
static void vel_controller(void)                                       
{ 
	/* 平衡控制：速度环 */	
  pidType.rol_encoder.expect = 0.0F;                              									//期望值0
  pidType.rol_encoder.feedback = 0.5F*(-encoderINFO.nb1_val + encoderINFO.nb2_val);	//编码器速度值作为反馈量，此处根据实际情况进行正负选择
  pid_controller(&pidType.rol_encoder);                           									//pid控制器  
	
	/* 转向控制：速度环 */
	float trun_ctrl = turn_round(BluetoothParseMsg.Xrocker, BluetoothParseMsg.Yrocker);
  pidType.yaw_encoder.expect = trun_ctrl;																						//期望值
  pidType.yaw_encoder.feedback = (-encoderINFO.nb1_val - encoderINFO.nb2_val);			//编码器速度差值作为反馈量，此处根据实际情况进行正负选择
	if( f_abs(trun_ctrl) > 5.0F ){
		pid_controller(&pidType.yaw_encoder);                           								//当有转向控制时才进行pid控制	
	}
	else{
		pidType.yaw_encoder.out = 0.0F;
	}
}
/**
  * @brief   内环角度控制器
  */
static void angle_controller(void)                                     
{
	/* 平衡控制：角度环 */
  pidType.rol_angle.expect = pidType.rol_encoder.out;          		//外环速度环输出给内环角度环作为期望值
  pidType.rol_angle.feedback = att.rol - att.rol_mechineMedian; 	//角度作为反馈量（实测角度-机械中值）
  pid_controller(&pidType.rol_angle);                             //pid控制器     
}
/**
  * @brief   内环角速度控制器
  */
static void gyro_controller(void)                                      
{                            
	/* 平衡控制：角速度环 */
  pidType.rol_gyro.expect = pidType.rol_angle.out;             		//外环输出作为内环期望值
  pidType.rol_gyro.feedback = Mpu.deg_s.y;                       	//角速度值作为反馈量
  pid_controller(&pidType.rol_gyro);                              //pid控制器  
}

/*
 * 函数名：ctr_bal
 * 描述  ：角度PD控制器
 * 输入  ：angle当前角度，gyro当前角速度
 * 返回  ：PID控制器输出
 */
int ctr_bal(float angle,float gyro)
{       
	ctr.exp[0] = 0;                                               //期望角度    
	ctr.bais[0] = (float)((angle-att.pit_mechineMedian) - ctr.exp[0]);	//角度偏差
	ctr.balance = bal.kp * ctr.bais[0] + gyro * bal.kd;           //角度平衡控制
	return ctr.balance;
}
/*
 * 函数名：ctr_vel
 * 描述  ：速度PI控制器
 * 输入  ：encoder_left编码器值A，encoder_right编码器值B
 * 返回  ：PID控制器输出
 */
#define BAIS_LIMIT vSPEED
int ctr_vel(int encoder_left,int encoder_right)
{  
    static float err_cur,err_last;

    err_last = ((encoder_left) + (encoder_right)) - 0;        //速度误差
    err_cur = err_cur * 0.8 + err_last * 0.2;         				//对偏差进行低通滤波
    ctr.bais[1] += err_cur;                                   //偏差累加和   
    ctr.bais[1] = ctr.bais[1] - go_straight(BluetoothParseMsg.Yrocker);	//遥控控制前后方向
    
    if(ctr.bais[1] > BAIS_LIMIT) ctr.bais[1] = BAIS_LIMIT;                    //限幅
    if(ctr.bais[1] <-BAIS_LIMIT) ctr.bais[1] =-BAIS_LIMIT; 
	
    ctr.velocity = err_cur * vel.kp + ctr.bais[1] * vel.ki + vel.kd * (err_cur - err_last);   //速度控制     
    return ctr.velocity;
}

/**
  * @brief   三环串级PID控制器运行
  */
void _controller_perform(void)                                  
{   
	/* 动量轮平衡控制 */
  vel_controller();                                           		//速度控制器
  angle_controller();                                         		//角度控制器
  gyro_controller();                                          		//角速度控制器    
	
	/* 直立平衡控制 */
	ctr.pwm[0] = ctr_bal(-att.pit,-Mpu.deg_s.x);                    //直立环控制器
	ctr.pwm[1] = ctr_vel(encoderINFO.nb3_val,0);                   //速度环控制器          
}



/**
  * @brief   检测是否倒下
  * @param   inAngleData 输入的参考角度值
  * @retval  detectionMark 返回是否倒下标志 1：倒下 0：正常
  */
static uint8_t detectionFallDown(float inAngleData)
{
  uint8_t detectionMark = 0;

  if( fabs(inAngleData) > 25.0f ) {
    detectionMark = 1;                                             //倒下则关闭输出标志置位                 
  }                                                            
  return detectionMark;     
}
/**
  * @brief   PWM输出
  * @param   pwm1，pwm2，pwm3 输入三路pwm
  */
static void pwmMotorOut(uint16_t pwm1 ,uint16_t pwm2,uint16_t pwm3)
{
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, limit_u16(pwm1 , 0 , 2000));	 //控制占空比
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, limit_u16(pwm2 , 0 , 2000));	 //控制占空比
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, limit_u16(pwm3 , 0 , 2000));	 //控制占空比
}

/* 根据输出正负来判断方向 */
static void dir_ctrl(int motor1,int motor2)
{
  if( motor1 > 0 ) {
		dir1Anticlockwise();
	}  
  else {
		dir1Clockwise();
	}
	
  if( motor2 > 0 ) {
		dir2Anticlockwise();
	}  
  else {
		dir2Clockwise();
	}
}

/*
 * 描述  ：前后平衡电机方向控制
 * 输入  ：motor：输入的pwm 
 */
void dir_forwardOrBackward(int motor)
{
    if( motor < 0 ) {
      AIN1_HIGH;
      AIN2_LOW;
    }
    else {
      AIN1_LOW;
      AIN2_HIGH;
    }
}

/**
  * @brief  控制器积分项清除
  */
static void clr_integral(void)
{
    clear_integral(&pidType.rol_encoder);         
    clear_integral(&pidType.rol_angle);                   
    clear_integral(&pidType.rol_gyro);  	
}
/*
 * 函数名：pickUpTheCar
 * 描述  ：小车被拿起
 * 输入  ：encoder电机编码器值
 * 返回  ：是否成功拿起
 */
static uint8_t is_StopTheCar = 1;
uint8_t pickUpTheCar(int encoder1,int encoder2,int encoder3)
{
	static uint8_t ret = 0;
	static uint16_t ticks = 0;
	/* 三电机任一电机速度超过设定速度持续一定时间 */
	if( abs(encoder1) > 250.0F || abs(encoder2) > 250.0F || abs(encoder3) > 1500.0F ){ 
		ticks++;
		if( ticks > 150 ){					/* 时间：150*5ms */
			ticks = 0;
			ret = 1;
		}
	}
	else{
		ticks = 0;
		ret = 0;
	}
	return ret;
}

/*
 * 函数名：putDownTheCar
 * 描述  ：小车被放下
 * 输入  ：angle当前角度，encoder电机编码器值，
 * 返回  ：是否成功放下
 */
uint8_t putDownTheCar(float angle1,float angle2,int encoder1,int encoder2,int encoder3)
{
    static uint8_t ret = 0 ; 
    static uint16_t ticks = 0;
    
    if( is_StopTheCar == 0 ) {					/* 判断当前是否已被放下，防止误触发 */
			return 0;
		}                                 	
    /* 小车静止，并且放置的角度在平衡位置±5度以内 */
    if( (abs(encoder1) == 0) && (abs(encoder2) == 0) && (abs(encoder3) == 0) && \
				(f_abs(angle1) < 10.0F) && (f_abs(angle2) < 10.0F) ) {
        ticks++;
        if( ticks > 300 ) {							/* 时间：300*5ms */
            ticks = 0;
            ret = 1;
        }
    }
    else {
        ticks = 0;
        ret = 0;
    }
    return ret;
}

/**
  * @brief   控制器输出
  */
void _controller_output(void)                                    
{ 
	if( 1 == pickUpTheCar(encoderINFO.nb1_val,encoderINFO.nb2_val,encoderINFO.nb3_val) ){
		is_StopTheCar = 1;
	}
	if( 1 == putDownTheCar(att.pit-att.pit_mechineMedian,att.rol-att.rol_mechineMedian,encoderINFO.nb1_val,encoderINFO.nb2_val,encoderINFO.nb3_val) ){
		is_StopTheCar = 0;	
	} 
	/* 倒下或者被拿起时停车 */ 
  if((detectionFallDown(att.pit) == 1) || (detectionFallDown(att.rol) == 1) || (1 == is_StopTheCar)) {	
		is_StopTheCar = 1;
    motor1.out = 0;                               										/* 清除电机PWM输出值 */
		motor2.out = 0;																										/* 清除电机PWM输出值 */
		motor3.out = 0;																										/* 清除电机PWM输出值 */
    clr_integral();   																								/* 清除积分 */
		for(uint8_t i = 0 ; i < 3 ; i++){
				ctr.bais[i] = 0;																							/* 清除偏差 */
		}
		Motor1Stop();																											/* 电机1失能 */
		Motor2Stop();																											/* 电机2失能 */
  }
  else{
		Motor1EN();																												/* 电机1使能 */
		Motor2EN();                                                       /* 电机2使能 */
		motor1.out =  pidType.rol_gyro.out + pidType.yaw_encoder.out;			/* 电机1输出匹配 */				
		motor2.out = -pidType.rol_gyro.out + pidType.yaw_encoder.out;			/* 电机2输出匹配 */		
		motor3.out = ctr.pwm[0] + ctr.pwm[1] ;                        		/* 电机3输出匹配 */   		
  }          
	if( 1 == cal_info.cal_cmd.enable){							/* 校准时使能 */
		motor1.out = 0;
		motor2.out = 0;
		motor3.out = 0;
	}
  dir_ctrl(motor1.out,motor2.out);                         						/* 方向判别 */
	dir_forwardOrBackward(motor3.out);           												/* 前后平衡电机方向判别 */
	pwmMotorOut( abs(motor1.out) ,abs(motor2.out) ,abs(motor3.out));		/* 电机输出 */
}  





