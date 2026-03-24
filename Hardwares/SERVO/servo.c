#include "servo.h"
#include "tim.h"

/**
  * @brief   舵机转向控制 舵机的控制一般需要一个20ms左右的时基脉冲，
  * 该脉冲的高电平部分一般为0.5ms-2.5ms范围内的角度控制脉冲部分，总间隔为2ms。
  * 以180度角度伺服为例，那么对应的控制关系是这样的：
  * 0.5ms------------0度； 
  * 1.0ms------------45度；
  * 1.5ms------------90度；
  * 2.0ms-----------135度；
  * 2.5ms-----------180度；
  * __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 5);  //0.5ms
  * __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 10); //1.0ms    
  * __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 15); //1.5ms    
  * __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 20); //2.0ms    
  * __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 25); //2.5ms
  */

void server_ctrl(uint8_t cmd )
{
    if(cmd == 0 ){
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 5);     //0.5ms
    }
    else {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 10);    //1ms
    }
}






