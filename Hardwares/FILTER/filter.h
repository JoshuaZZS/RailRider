/**
  ******************************************************************************
  * @file    filter.h
  * @author  Mr.Lin
  * @version V2022.04.01
  * @date    01-Apr-2022
  * @brief   用户进行滤波处理相关的功能函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H

#include "imath.h"

typedef struct
{
  float input_data[3];
  float output_data[3];
}_Butterworth_data;
typedef struct
{
  const float a[3];
  const float b[3];
}_Butterworth_parameter;

float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter);
void get_iir_factor(float *out_factor,float Time, float Cut_Off);
void iir_lpf_run(SI_F_XYZ *input_data,SI_F_XYZ *output_data,float lpf_factor);
void butterworth_lpf_run(SI_F_XYZ *input_Data , SI_F_XYZ *output_Data , _Butterworth_parameter lpf_parameter);

#endif /* __FILTER_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
