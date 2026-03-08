/**
  ******************************************************************************
  * @file    ano.c
  * @author  Mr.Lin
  * @version V2022.04.19
  * @date    19-Apr-2022
  * @brief   与匿名地面站上位机进行数据通信相关的协议代码及相关功能函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

#include "ano.h"
#include "stdint.h"
#include "mpu6050.h"
#include "imu.h"
#include "usart.h"
#include "analog.h"
#include "controller.h"
#include "queue_msg.h"
#include "calibration.h"
#include "record.h"
#include "string.h"
#include "pid.h"
#include "encoder.h"
union dt_flag_t dtsend;				                  //需要发送数据的标志 
uint8_t data_to_send[100];                  //发送数据缓存
uint8_t USART_RX_DATA[USART_RX_LEN];        //接收数据缓存

/* 将大于一个字节的数据拆分成多个字节发送 */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/* 向匿名上位机发送姿态角，锁定状态 */
static void ANO_DT_Send_Status(void)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	volatile int32_t _temp2;
	uint8_t sum = 0;
	uint8_t i;
  
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(att.rol*100);                     //横滚角
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
	_temp = (int)(att.pit*100);                     //俯仰角    
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
	_temp = (int)(att.yaw*100);                     //偏航角
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp2 = (int32_t)(100*99.99);       		        //高度 
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

  data_to_send[_cnt++]=0x01;  					          //飞行模式    01：姿态  02：定高  03：定点
  data_to_send[_cnt++]=0x01;                      //锁定状态

	data_to_send[3] = _cnt-4;
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
    
  Usart3_DMA_Sent(data_to_send,_cnt);   //发送
}
/* 向匿名上位机发送传感器原始数据 */
static void ANO_DT_Send_Senser( int16_t a_x,int16_t a_y,int16_t a_z,\
                                int16_t g_x,int16_t g_y,int16_t g_z,\
                                int16_t m_x,int16_t m_y,int16_t m_z)
{
  uint8_t _cnt=0;
  volatile int16_t _temp;
  uint8_t sum = 0;
  uint8_t i=0;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x02;
  data_to_send[_cnt++]=0;

  _temp = a_x;    
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = a_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  _temp = g_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = g_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  _temp = m_x;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_y;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = m_z;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  data_to_send[3] = _cnt-4;

  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++] = sum;
    
  Usart3_DMA_Sent(data_to_send,_cnt);   //发送
}
/* 遥控器通道数据 */
static void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,\
                               uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
  uint8_t _cnt=0;
  uint8_t i=0;
  uint8_t sum = 0;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x03;
  data_to_send[_cnt++]=0;

  data_to_send[_cnt++]=BYTE1(thr);
  data_to_send[_cnt++]=BYTE0(thr);
  data_to_send[_cnt++]=BYTE1(yaw);
  data_to_send[_cnt++]=BYTE0(yaw);
  data_to_send[_cnt++]=BYTE1(rol);
  data_to_send[_cnt++]=BYTE0(rol);
  data_to_send[_cnt++]=BYTE1(pit);
  data_to_send[_cnt++]=BYTE0(pit);

  data_to_send[_cnt++]=BYTE1(aux1);
  data_to_send[_cnt++]=BYTE0(aux1);
  data_to_send[_cnt++]=BYTE1(aux2);
  data_to_send[_cnt++]=BYTE0(aux2);
  data_to_send[_cnt++]=BYTE1(aux3);
  data_to_send[_cnt++]=BYTE0(aux3);
  data_to_send[_cnt++]=BYTE1(aux4);
  data_to_send[_cnt++]=BYTE0(aux4);
  data_to_send[_cnt++]=BYTE1(aux5);
  data_to_send[_cnt++]=BYTE0(aux5);
  data_to_send[_cnt++]=BYTE1(aux6);
  data_to_send[_cnt++]=BYTE0(aux6);

  data_to_send[3] = _cnt-4;

  sum = 0;
  for(i=0;i<_cnt;i++)
    sum += data_to_send[i];
  data_to_send[_cnt++]=sum;

  Usart3_DMA_Sent(data_to_send,_cnt);   //发送
}
/* 电压电流数据发送至匿名上位机 */ 
static void ANO_DT_Send_Power(float votage, float current)
{
	uint8_t _cnt=0;
	uint16_t temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = (uint16_t)100*votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = (uint16_t)100*current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
  Usart3_DMA_Sent(data_to_send,_cnt);   //发送
}
/* 
 * 用户自定义数据发送：
 * 1-5：int16t类型数据     
 * 6-10：float类型数据 
 */
static void ANO_DT_Send_User( int16_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,
                              float user6, float user7, float user8, float user9, float user10,
                              float user11,float user12,float user13,float user14,float user15)
{
  uint8_t _cnt=0;
  volatile int16_t _temp;
  float _temp_f;
  
  uint8_t sum = 0;
  uint8_t i=0;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0xF1;
  data_to_send[_cnt++]=0;
  
  //1-5  int16t类型数据
  _temp = user1;    
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  
  _temp = user2;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = user3;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = user4;    
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);
  _temp = user5;
  data_to_send[_cnt++]=BYTE1(_temp);
  data_to_send[_cnt++]=BYTE0(_temp);

  //6-10 ：float类型数据
  _temp_f = user6;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user7;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user8;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user9;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user10;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);

  _temp_f = user11;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user12;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user13;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user14;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);
  _temp_f = user15;
  data_to_send[_cnt++]=BYTE3(_temp_f);
  data_to_send[_cnt++]=BYTE2(_temp_f);
  data_to_send[_cnt++]=BYTE1(_temp_f);
  data_to_send[_cnt++]=BYTE0(_temp_f);

  data_to_send[3] = _cnt-4;

  sum = 0;
  for(i=0;i<_cnt;i++)
      sum += data_to_send[i];
  data_to_send[_cnt++] = sum;
  
  Usart3_DMA_Sent(data_to_send,_cnt);   //发送
}

static void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,\
                              float p2_p,float p2_i,float p2_d,\
                              float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt = 0;
	uint16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++] = sum;

	Usart3_DMA_Sent(data_to_send, _cnt);
}

/*
在上位机对下位机进行操作时，下位机做出对应的反馈信息
 MSG_ID:                 MSG_DATA:
01:加速度               01:校准成功
02:陀螺仪               E1:校准失败
03:罗盘                 31:设置成功
30:无线定位模块         32:设置成功2
40:匿名数传             A1:恢复默认成功
 */
static void ANO_DT_Send_MSG(uint8_t MSG_ID, uint8_t MSG_DATA)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;                  //长度
	data_to_send[4]=MSG_ID;             //功能字
	data_to_send[5]=MSG_DATA;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

  Usart3_DMA_Sent(data_to_send,7);    //发送
}

/* 发送数据 */
void ANO_DMA_SEND_DATA(void)
{
  static uint8_t ANO_debug_cnt = 0;
  ANO_debug_cnt++;
  if( ANO_debug_cnt == 1 ){
    ANO_DT_Send_Status();
  }
  else if( ANO_debug_cnt == 2 ){  
    ANO_DT_Send_Senser((int16_t)acc_raw.x,(int16_t)acc_raw.y,(int16_t)acc_raw.z, 
                       (int16_t)gyro_raw_f.x,(int16_t)gyro_raw_f.y,(int16_t)gyro_raw_f.z,
                       (int16_t)0,(int16_t)0,(int16_t)0);
  }
  else if( ANO_debug_cnt == 3 ){
    ANO_DT_Send_RCData(1100,1200,1300,1400,1500,1600,1700,1800,1900,1100);
  }
  else if( ANO_debug_cnt == 4 ){
    ANO_DT_Send_Power( adc_info.voltage,56.78F );
  }
  else if( ANO_debug_cnt == 5 ){
    ANO_DT_Send_User((int16_t)encoderINFO.nb1_val,motor1.out,0,0,0,
                    0,(float)encoderINFO.nb1_val,0,0,0,
                    0,0,0,0,0);
  }
  else if( ANO_debug_cnt == 6 ){
    if( dtsend.bit.pid4 ){
      dtsend.bit.pid4 = 0;      
    }
    else if( dtsend.bit.pid5 ){
      dtsend.bit.pid5 = 0;
    }
    else if( dtsend.bit.pid6 ){
      dtsend.bit.pid6 = 0;
      ANO_DT_Send_PID(6,  \
                          pidType.rol_angle.kp , \
                          pidType.rol_angle.ki , \
                          pidType.rol_angle.kd , \
													pidType.rol_encoder.kp , \
                          pidType.rol_encoder.ki , \
                          pidType.rol_encoder.kd , \
                          pidType.rol_gyro.kp , \
                          pidType.rol_gyro.ki , \
                          pidType.rol_gyro.kd);   
    }
    if( cal_info.cal_result.gyro == 1 ) {
      cal_info.cal_result.gyro = 0;
      ANO_DT_Send_MSG(SENSOR_GYRO,CAL_SUCCESS);     //返回校准成功信息给上位机
    }
    else if( cal_info.cal_result.gyro == 2 ) {
      cal_info.cal_result.gyro = 0;
      ANO_DT_Send_MSG(SENSOR_GYRO,CAL_FAIL);        //反馈校准失败信息给上位机
    }
    else{}
  }
  else {
    ANO_debug_cnt = 0;
  }
}

static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;                  //长度
	data_to_send[4]=head;               //功能字
	data_to_send[5]=check_sum;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++){
		sum += data_to_send[i];
	}
	data_to_send[6]=sum;

  Usart3_DMA_Sent(data_to_send,7);   //发送
}

/* 协议解析 */
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{  
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++){
    sum += *(data_buf+i);        
  }
	if(!(sum==*(data_buf+num-1))){
    return;	
  }
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	{
    return;		
  }

	if(*(data_buf+2)==0X01) {                 //功能字01 
    switch ( *(data_buf+4) ){
    case 0x01:                              //ACC校准
      break;                      
    case 0x02: cal_info.cal_cmd.gyro = 1;   //GYRO校准
      break;
    case 0x21: cal_info.cal_cmd.acc = 1;    //六面校准：水平放置  
      break;
    case 0x22: cal_info.cal_cmd.acc = 5;    //六面校准：机头朝上垂直放置  
      break;
    case 0x23: cal_info.cal_cmd.acc = 3;    //六面校准：左侧朝上垂直放置  
      break;
    case 0x24: cal_info.cal_cmd.acc = 4;    //六面校准：右侧朝上垂直放置  
      break;
    case 0x25: cal_info.cal_cmd.acc = 6;    //六面校准：机尾朝上垂直放置  
      break;
    case 0x26: cal_info.cal_cmd.acc = 2;    //六面校准：背面朝上水平放置  
      break;
    case 0x20: cal_info.cal_cmd.acc = 0;    //退出六面校准  
      break;
    default:  break;
    }
	}                                         
	else if(*(data_buf+2)==0X02) {          	//功能字02
		if(*(data_buf+4)==0X01) {               //上位机读取pid请求
			dtsend.bit.pid1 = 1;
      dtsend.bit.pid2 = 1;
      dtsend.bit.pid3 = 1;
      dtsend.bit.pid4 = 1;
      dtsend.bit.pid5 = 1;
      dtsend.bit.pid6 = 1;
		}
		else if(*(data_buf+4)==0X02) {     			//读取飞行模式设置请求
		}
		else if(*(data_buf+4)==0XA0) {         	//读取版本信息
			dtsend.bit.version = 1;
		}
		else if(*(data_buf+4)==0XA1) {         	//恢复默认参数
      pid_init();                           
		}
		else{}
	}
  else if(*(data_buf+2)==0X10) {					//设置PID组0
		dbg("pid0 ack ok\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
  }
  else if(*(data_buf+2)==0X11) {					//设置PID组1
		dbg("pid1 ack ok\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
  }
	else if(*(data_buf+2)==0X12) {					//设置PID组2
		dbg("pid2 ack ok\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);  
	}
	else if(*(data_buf+2)==0X13) {					//设置PID组3
		dbg("pid3 ack ok\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
  }
  else if(*(data_buf+2)==0X14) {					//设置PID组4
		dbg("pid4 ack ok\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
  }
  else if(*(data_buf+2)==0X15) {					//设置PID组5
		dbg("pid5 ack ok\n");
		ANO_DT_Send_Check(*(data_buf+2),sum);
    pidType.rol_angle.kp = 0.001F * ( (uint16_t) ((*(data_buf+4)<<8) | *(data_buf+5)) );
    pidType.rol_angle.ki = 0.001F * ( (uint16_t) ((*(data_buf+6)<<8) | *(data_buf+7)) );
    pidType.rol_angle.kd = 0.001F * ( (uint16_t) ((*(data_buf+8)<<8) | *(data_buf+9)) );
    pidType.rol_encoder.kp = 0.001F * ( (uint16_t) ((*(data_buf+10)<<8) | *(data_buf+11)) );
    pidType.rol_encoder.ki = 0.001F * ( (uint16_t) ((*(data_buf+12)<<8) | *(data_buf+13)) );
    pidType.rol_encoder.kd = 0.001F * ( (uint16_t) ((*(data_buf+14)<<8) | *(data_buf+15)) );
    pidType.rol_gyro.kp = 0.001F * ( (uint16_t) ((*(data_buf+16)<<8) | *(data_buf+17)) );
    pidType.rol_gyro.ki = 0.001F * ( (uint16_t) ((*(data_buf+18)<<8) | *(data_buf+19)) );
    pidType.rol_gyro.kd = 0.001F * ( (uint16_t) ((*(data_buf+20)<<8) | *(data_buf+21)) );
  }
	else{}
		
	
}

/* 协议解析 */
void ANO_DT_Data_Receive_Prepare(uint8_t data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if( state == 0 && data == 0xAA ){
		state = 1;
		RxBuffer[0] = data;
	}
	else if( state == 1 && data == 0xAF ){
		state = 2;
		RxBuffer[1] = data;
	}
	else if( state == 2 && data < 0XF1 ){
		state = 3;
		RxBuffer[2] = data;
	}
	else if( state == 3 && data < 50 ){
		state = 4;
		RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if( state == 4 && _data_len > 0 ){
		_data_len--;
		RxBuffer[4+_data_cnt++] = data;
		if( _data_len == 0 )
			state = 5;
	}
	else if( state == 5 ){
		state = 0;
		RxBuffer[4+_data_cnt] = data;
		ANO_DT_Data_Receive_Anl(RxBuffer , _data_cnt+5);
	}
	else{
		state = 0;    
  }
}


/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/

