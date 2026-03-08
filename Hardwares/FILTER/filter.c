/**
  ******************************************************************************
  * @file    filter.c
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

/* Includes ------------------------------------------------------------------*/
#include "filter.h"
#include "mpu6050.h"

 /**
  * @brief   二阶巴特沃斯滤波器原型
  * @param   now_input：输入数据
  * @param   buffer：中间数据缓存
  * @param   parameter：滤波参数

  */
float butterworth_lpf(float now_input,_Butterworth_data *buffer, _Butterworth_parameter *parameter)
{
  buffer->input_data[2] = now_input;

  /* Butterworth LPF */
  buffer->output_data[2] =   parameter->b[0] * buffer->input_data[2]
                           + parameter->b[1] * buffer->input_data[1]
                           + parameter->b[2] * buffer->input_data[0]
                           - parameter->a[1] * buffer->output_data[1]
                           - parameter->a[2] * buffer->output_data[0];
  /* x(n) 保存 */
  buffer->input_data[0] = buffer->input_data[1];
  buffer->input_data[1] = buffer->input_data[2];
  /* y(n) 保存 */
  buffer->output_data[0] = buffer->output_data[1];
  buffer->output_data[1] = buffer->output_data[2];

  return buffer->output_data[2];
}

 /**
  * @brief   二阶巴特沃斯滤波器参数选择（200hz采样频率-5hz截止频率）
  * @param   none
  * @retval  none
  */
_Butterworth_parameter lpf_5hz_parameter =
{
  //200hz---1hz
//  1,   -1.955578240315,   0.9565436765112,
//  0.000241359049042, 0.000482718098084, 0.000241359049042
  //200hz---2hz
//  1,   -1.911197067426,   0.9149758348014,
//  0.0009446918438402,  0.00188938368768,0.0009446918438402
    //200hz---5hz
    1,                  -1.778631777825,    0.8008026466657,
    0.005542717210281,   0.01108543442056,  0.005542717210281
    //200hz---10hz
//    1,   -1.561018075801,   0.6413515380576,
//    0.02008336556421,  0.04016673112842,  0.02008336556421
    //200hz---15hz
//    1,   -1.348967745253,   0.5139818942197,
//    0.04125353724172,  0.08250707448344,  0.04125353724172
    //200hz---20hz
//    1,    -1.14298050254,   0.4128015980962,
//    0.06745527388907,   0.1349105477781,  0.06745527388907
    //200hz---30hz
//    1,  -0.7477891782585,    0.272214937925,
//    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   Middle_butter_data[3];
/*
 * 函数名：acc_butterworth_lpf
 * 描述  ：巴特沃斯低通滤波
 * 输入  ：acc_in滤波前的加速度首地址，acc_out滤波后的输出加速度数据首地址     
 * 返回  ：     
 */
 /**
  * @brief   巴特沃斯低通滤波
  * @param   *input_Data：滤波前的输入数据首地址
  * @param   *output_Data：滤波后的输出数据首地址
  */
void butterworth_lpf_run(SI_F_XYZ *input_Data , SI_F_XYZ *output_Data , _Butterworth_parameter lpf_parameter)
{
  output_Data->x = butterworth_lpf( input_Data->x , &Middle_butter_data[0] , &lpf_parameter );
  output_Data->y = butterworth_lpf( input_Data->y , &Middle_butter_data[1] , &lpf_parameter );
  output_Data->z = butterworth_lpf( input_Data->z , &Middle_butter_data[2] , &lpf_parameter );    
}

/**
  * @brief   求取IIR滤波器的滤波因子
  * @param   out_factor：滤波因子首地址
  * @param   Time：任务执行周期
  * @param   Cut_Off：滤波截止频率
  */
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
	*out_factor = Time /( Time + 1 / (2.0f * PI * Cut_Off) );
}
/**
  * @brief   IIR低通滤波器
  * @param   *acc_in：输入三轴数据指针变量
  * @param   *acc_out：输出三轴数据指针变量
  * @param   lpf_factor：滤波因数
  */
void iir_lpf_run(SI_F_XYZ *input_data,SI_F_XYZ *output_data,float lpf_factor)
{
	output_data->x = output_data->x + lpf_factor * (input_data->x - output_data->x); 
	output_data->y = output_data->y + lpf_factor * (input_data->y - output_data->y); 
	output_data->z = output_data->z + lpf_factor * (input_data->z - output_data->z); 
}

/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
