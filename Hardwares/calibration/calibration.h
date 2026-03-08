/**
  ******************************************************************************
  * @file    calibration.h
  * @author  Mr.Lin
  * @version V2022.04.01
  * @date    01-Apr-2022
  * @brief   传感器校准
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include "imath.h"

#define CAL_NB 10
typedef enum
{
    false = 0,
    true = !false
}bool;

struct _CALIBRATE
{
	uint8_t acc :3;    
	uint8_t gyro :2;
  uint8_t mechMidan :2;
	uint8_t enable :1;	
};

struct CAL_INFO
{
  struct _CALIBRATE cal_cmd;
  struct _CALIBRATE cal_result;
};

typedef struct  
{
  float K[3];
  float B[3];
}ACC_CALIBRATE;

typedef struct  
{
  float pit;
  float rol;
  float yaw;
}ANGLE_MECHINE_MEDIAN;

struct SENSOR_CALIBRATE
{
  SI_F_XYZ gyro;
  ACC_CALIBRATE acc;
  ANGLE_MECHINE_MEDIAN mechMedian;
};

void zero_calibration(SI_F_XYZ *pin_raw_data , SI_F_XYZ *pout_data );
void machineMedian_calibration(uint8_t cmd, float *pin_data , float *pout_data);
int read_cal_para(void);

extern struct CAL_INFO cal_info;
extern struct SENSOR_CALIBRATE sensor_cal;

#endif /* __CALIBRATION_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
