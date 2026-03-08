/**
  ******************************************************************************
  * @file    analog.h
  * @author  Mr.Lin
  * @version V2.10.28
  * @date    19-Apr-2022
  * @brief   模拟量数据相关函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANALOG_H
#define __ANALOG_H

#include "stm32f1xx_hal.h"

#define SAMPLE_NB 20

typedef struct
{
  uint16_t vbat_raw ; 
  float voltage ; 
}_ADC_INFO;

extern _ADC_INFO adc_info ;
extern uint16_t adc_converted_value[SAMPLE_NB] ;

uint16_t get_voltagedata(void);



#endif /* __ANO_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
