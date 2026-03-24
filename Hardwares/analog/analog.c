
/**
  ******************************************************************************
  * @file    analog.c
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

#include "analog.h"

_ADC_INFO adc_info = {0} ;
uint16_t adc_converted_value[SAMPLE_NB] ;

/**
 * @brief   读取电压adc数值
 * @retval  电压adc平均值
 */
uint16_t get_voltagedata(void)
{
  uint32_t i = 0, adc_val = 0;
  
  for( i = 0; i < SAMPLE_NB; ++i ){
    adc_val += adc_converted_value[i];      /* 将adc值连续存入20个数据到数组中，然后求取平均值 */
  }
  adc_val = adc_val / SAMPLE_NB;
  return adc_val;
}

/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/

