/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  Mr.Lin
  * @version V2022.03.31
  * @date    31-Mar-2022
  * @brief   六轴姿态传感器数据读取及相关处理
  * @Tips    [ C - Standardization Tool ] [ By weimin ]
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU6050_H
#define __MPU6050_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"  
#include "imath.h"


#define PI                      3.1415926535898f
#define gyro_raw_to_deg_s       0.0609756097561f   /* +-250°/s:131LSB/°/s   +-500°/s:65.5LSB/°/s   +-1000°/s:32.8LSB/°/s    +-2000°/s:16.4LSB/°/s(本次所用) */
#define acc_raw_to_g            0.000244140625f    /* +-2g : 16384LSB/g     +-4g : 8192LSB/g   +-8g : 4096LSB/g(本次所用)   +-16g : 2048LSB/g */
#define deg_to_rad              (PI / 180.0f)
#define rad_to_angle            (180.0f / PI)                    
#define gyro_raw_to_radian_s	(gyro_raw_to_deg_s * deg_to_rad)
#define accmax_1g      4096
#define gravity_mss    9.80665f                    /* acceleration due to gravity in m/s/s */
#define acc_to_1g      gravity_mss / accmax_1g
#define one_g_to_acc   accmax_1g / gravity_mss

typedef struct 
{
  SI_F_XYZ deg_s;
  SI_F_XYZ rad_s;
  SI_F_XYZ acc_g;
  float att_acc_factor;
  float fix_acc_factor;
}_Mpu6050_data;

void Mpu6050_init(void);
uint8_t MPU6050ReadID(void);
void mpu6050_loop(void);

extern _Mpu6050_data Mpu;
extern S16_XYZ  acc_raw;
extern S16_XYZ  gyro_raw;
extern SI_F_XYZ gyro_raw_cal;
extern SI_F_XYZ gyro_offset;
extern SI_F_XYZ gyro_raw_f;

#endif /* __MPU6050_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
