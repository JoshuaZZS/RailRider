/**
  ******************************************************************************
  * @file    vofa.c
  * @author  Mr.Lin
  * @version V2022.04.19
  * @date    19-Apr-2022
  * @brief   与VOFA上位机进行数据通信相关的协议代码及相关功能函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#include "vofa.h"
#include "imu.h"
#include "queue_msg.h"
#include "mpu6050.h"
#include "string.h"
#include "pid.h"
#include "controller.h"
#include "bluetooth.h"
#include "ano.h"
#include "calibration.h"
#include "encoder.h"

struct Frame vofa_frame = {
	.tail[0] =  0x00,
	.tail[1] =  0x00,
	.tail[2] =  0x80,
	.tail[3] =  0x7f
};

void vofa_transform_upload(void)
{
	static uint8_t data_to_send[CH_COUNT*4+4];     
    uint8_t cnt = 0;
	/* 三轴加速度、角速度、角度 */
	vofa_frame.fdata[cnt++] = (float)acc_raw.x;
	vofa_frame.fdata[cnt++] = (float)acc_raw.y;
	vofa_frame.fdata[cnt++] = (float)acc_raw.z;
	vofa_frame.fdata[cnt++] = gyro_raw_f.x;
	vofa_frame.fdata[cnt++] = gyro_raw_f.y;
	vofa_frame.fdata[cnt++] = gyro_raw_f.z;
	vofa_frame.fdata[cnt++] = att.pit;
	vofa_frame.fdata[cnt++] = att.rol;
	vofa_frame.fdata[cnt++] = att.yaw;
    /* 机械中值 */
	vofa_frame.fdata[cnt++] = att.rol_mechineMedian;
    /* 三电机速度值 */
	vofa_frame.fdata[cnt++] = (float)encoderINFO.nb1_val;
	vofa_frame.fdata[cnt++] = (float)encoderINFO.nb2_val;
	vofa_frame.fdata[cnt++] = (float)encoderINFO.nb3_val;
    /* 三电机输出值 */
	vofa_frame.fdata[cnt++] = (float)motor1.out;
	vofa_frame.fdata[cnt++] = (float)motor2.out;
	vofa_frame.fdata[cnt++] = (float)motor3.out;
    /*备用通道*/
	vofa_frame.fdata[cnt++] = (float)att.pit_mechineMedian;
	vofa_frame.fdata[cnt++] = (float)0;

	memcpy(data_to_send,&vofa_frame,sizeof(vofa_frame));
	Usart3_DMA_Sent(data_to_send,sizeof(vofa_frame));      
}


/* 协议解析 */
static void vofa_parse_download(uint8_t *data_buf,uint8_t lens)
{  
	float rx_rol_machMedian;
	float rx_pit_machMedian;
	float rx_enable = 0.0F;
    if(!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))	{       /* 帧头判断 */
        return;		
    }
    if( *(data_buf + lens - 1) != ( *(data_buf + 2) + 0x10 ) ){     /* 判断首尾帧类型 */
        return;
    }
    uint8_t pid_type = *(data_buf + 2);
    switch( pid_type ){                                             /* 接收参数类型 */   

	/* 飞轮平衡参数（角度环） angle */		
    case 0xA1:pidType.rol_angle.kp = hex2float(&data_buf[3]);    break;
    case 0xA2:pidType.rol_angle.ki = hex2float(&data_buf[3]);    break;
    case 0xA3:pidType.rol_angle.kd = hex2float(&data_buf[3]);    break;
#if 1
    /* 飞轮平衡参数（速度环） encoder */	
    case 0xA4:pidType.rol_encoder.kp = hex2float(&data_buf[3]);  break;
    case 0xA5:pidType.rol_encoder.ki = hex2float(&data_buf[3]);  break;
    case 0xA6:pidType.rol_encoder.kd = hex2float(&data_buf[3]);  break;
#else
 	/* 飞轮转向参数 yaw（仅速度环） */		
    case 0xA4:pidType.yaw_encoder.kp = hex2float(&data_buf[3]);  break;
    case 0xA5:pidType.yaw_encoder.ki = hex2float(&data_buf[3]);  break;
    case 0xA6:pidType.yaw_encoder.kd = hex2float(&data_buf[3]);  break;
#endif
    /* 飞轮平衡参数（角速度环） gyro */	
    case 0xA7:pidType.rol_gyro.kp = hex2float(&data_buf[3]);     break;
    case 0xA8:pidType.rol_gyro.ki = hex2float(&data_buf[3]);     break;
    case 0xA9:pidType.rol_gyro.kd = hex2float(&data_buf[3]);     break;  

	/* 直立平衡参数（直立环 + 速度环）*/
    case 0xB1:bal.kp = hex2float(&data_buf[3]); break;
    case 0xB2:bal.ki = hex2float(&data_buf[3]); break;
    case 0xB3:bal.kd = hex2float(&data_buf[3]); break;
    case 0xB4:vel.kp = hex2float(&data_buf[3]); break;
    case 0xB5:vel.ki = hex2float(&data_buf[3]); break;
    case 0xB6:vel.kd = hex2float(&data_buf[3]); break;

    /* 传感器校准 */
    case 0xC1:  cal_info.cal_cmd.gyro = 1;   break;     /* GYRO校准 */
    case 0xC2:                                          /* 机械中值设置 */
        cal_info.cal_cmd.mechMidan = 1;
        rx_rol_machMedian = hex2float(&data_buf[3]); 
        machineMedian_calibration(0xC2,&rx_rol_machMedian , &att.rol_mechineMedian);
    break;
    case 0xC4:                                          /* 机械中值设置 */
        cal_info.cal_cmd.mechMidan = 1;
        rx_pit_machMedian = hex2float(&data_buf[3]); 
        machineMedian_calibration(0xC4,&rx_pit_machMedian , &att.pit_mechineMedian);
    break;
    case 0xC3:                                          /* 校准使能设置 */
        rx_enable = hex2float(&data_buf[3]); 
				cal_info.cal_cmd.enable = (uint8_t)rx_enable;
    default:break;
    }
}

void vofa_usart_rcvmsg(void)
{
    if( Usart3_Str.RecvFlags == 1 ) { 
        dbg("\nrvofa lens:%d\r\n", Usart3_Str.RecvLens);
        print_arry("vofa rbuf",Usart3_Str.RecvBuff,Usart3_Str.RecvLens,0);
        vofa_parse_download(Usart3_Str.RecvBuff ,Usart3_Str.RecvLens);      
        memset (Usart3_Str.RecvBuff , 0,Usart3_Str.RecvLens);                   
        Usart3_Str.RecvLens = 0;                                                
        Usart3_Str.RecvFlags = 0;                                               
    }   
}




/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
