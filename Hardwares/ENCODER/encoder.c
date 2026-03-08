#include "encoder.h"
#include "tim.h"

_ENCODER_INFO encoderINFO = { 0 } ;

/**
  * @brief   编码值处理，将原本反转由最大减小的正数数据转变为从0开始减小到负的数据
  * @param   *ecdData 编码数据指针变量
  */
static void EncoderManage(int *ecdData)
{
  //编码器反向时，转为与正向时对应的负数
  if(*ecdData > FULL_ENCODER * 0.5f){
    *ecdData = *ecdData - FULL_ENCODER ; 
  }
  else {
    *ecdData = *ecdData;
  }
}

/**
  * @brief   读取编码器转速值
  */
uint8_t readEncoderValue(void)
{
  encoderINFO.dir1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);      //读取编码转动的方向(正常转动才能判断正确，此处未使用，低速换向会干扰数据判断)
  encoderINFO.nb1_val = __HAL_TIM_GET_COUNTER(&htim2);            //读取编码值
  __HAL_TIM_SET_COUNTER(&htim2,0);                                //清楚编码值
  EncoderManage(&encoderINFO.nb1_val) ;                           //将电机反向转动的编码值转为负的数据

  encoderINFO.dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);      //读取编码转动的方向(正常转动才能判断正确，此处未使用，低速换向会干扰数据判断)
  encoderINFO.nb2_val = __HAL_TIM_GET_COUNTER(&htim3);            //读取编码值
  __HAL_TIM_SET_COUNTER(&htim3,0);                                //清楚编码值
  EncoderManage(&encoderINFO.nb2_val) ;                           //将电机反向转动的编码值转为负的数据
    
  encoderINFO.dir3 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);      //读取编码转动的方向(正常转动才能判断正确，此处未使用，低速换向会干扰数据判断)
  encoderINFO.nb3_val = __HAL_TIM_GET_COUNTER(&htim4);            //读取编码值
  __HAL_TIM_SET_COUNTER(&htim4,0);                                //清楚编码值
  EncoderManage(&encoderINFO.nb3_val) ;                           //将电机反向转动的编码值转为负的数据	
  return 0;
}





