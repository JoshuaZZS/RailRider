/**
  ******************************************************************************
  * @file    imath.h
  * @author  Mr.Lin
  * @version V2022.04.01
  * @date    01-Apr-2022
  * @brief   用户进行的相关数学运算功能函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMATH_H
#define __IMATH_H

#include "stdint.h"



typedef struct
{
	signed short x;
	signed short y;
	signed short z;
}S16_XYZ;

typedef struct
{
	float x;
	float y;
	float z;
}SI_F_XYZ;

union _H2F{
    float fdata;
    uint8_t farray[4];
};
float hex2float(uint8_t *buf);
float char2float(uint8_t *buf);
float invSqrt(float x);
uint16_t limit_u16(uint16_t in ,uint16_t min ,uint16_t max);
uint8_t verify_result(SI_F_XYZ *_in_data , float templt);
void sif_xyz_memcopy(SI_F_XYZ *_out_data,SI_F_XYZ *_in_data);
void  sif_xyz_memset(SI_F_XYZ *_in_data,float value);
float f_abs(float f);
void delay(uint32_t time);
#endif /* __IMATH_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
