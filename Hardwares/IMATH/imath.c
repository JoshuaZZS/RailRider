/**
  ******************************************************************************
  * @file    imath.c
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

/* Includes ------------------------------------------------------------------*/
#include "imath.h"
#include "math.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "queue_msg.h"

// Fast inverse square-root
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

uint16_t limit_u16(uint16_t in ,uint16_t min ,uint16_t max)
{
  if(in < min){
    in = min;
  }
  else if(in > max){
    in = max;
  }
  else{;}
  
	return in;
}

/*
 * 函数名：f_abs
 * 描述  ：浮点型数据绝对值
 * 输入  ：f浮点数据 
 * 返回  ：绝对值 
 */
float f_abs(float f)
{
  f = f < 0 ? -f : f;
	return f;
}

/**
  * @brief   浮点数据比较判断进行标志返回
  * @param   *_in_data 输入被比较的三轴数据指针变量
  * @param   templt 相比较的参考值
  * @retval  0 在参考值范围内返回OK，1 不在参考范围内返回ERROR
  */
uint8_t verify_result(SI_F_XYZ *_in_data , float templt)
{
  uint8_t ret = 0;
  if((fabs(_in_data->x) > templt) || (fabs(_in_data->y) > templt) || (fabs(_in_data->z) > templt)){
    ret = 1;
  }
  return ret;
}

/**
  * @brief   三轴数据进行赋值操作
  * @param   *_out_data 输出数据三轴指针变量
  * @param   *_in_data 输入三轴数据指针变量
  */
void sif_xyz_memcopy(SI_F_XYZ *_out_data,SI_F_XYZ *_in_data)
{
  _out_data->x = _in_data->x;
  _out_data->y = _in_data->y;
  _out_data->z = _in_data->z;
}
/*
 * 函数名：set_value
 * 描述  ：给数据赋值
 * 输入  ：_in_dat输入数据首地址， value所需要赋的值
 */
void  sif_xyz_memset(SI_F_XYZ *_in_data,float value)
{
    _in_data->x = value;
    _in_data->y = value;
    _in_data->z = value;
}

/* 四字节16进制hex转为float类型数据：联合体方式 */
float hex2float(uint8_t *buf)
{
		union _H2F h2f;	
    h2f.farray[0] = buf[0];
    h2f.farray[1] = buf[1];
    h2f.farray[2] = buf[2];
    h2f.farray[3] = buf[3];
    return h2f.fdata;
}

/* 四字节16进制hex转为float类型数据：取地址方式 */
float char2float(uint8_t *buf)
{
  float fdata;
  *((uint8_t *)(&fdata)) = buf[0];
  *((uint8_t *)(&fdata) + 1) = buf[1];
  *((uint8_t *)(&fdata) + 2) = buf[2];
  *((uint8_t *)(&fdata) + 3) = buf[3];
  return fdata;
}

void delay(uint32_t time)
{
	uint16_t tick = 100;
	for(int i=0;i<time;i++){
    while (tick--);		
	}
}

/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
