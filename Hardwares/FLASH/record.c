/**
  ******************************************************************************
  * @file    record.c
  * @author  Mr.Lin
  * @version V2022.04.19
  * @date    19-Apr-2022
  * @brief   数据读写相关
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

#include "record.h"
#include "pid.h"

struct _para_group fr_para = {0};
struct _para_group *wfr_para = &fr_para;
const struct _para_group *rfr_para = (const struct _para_group*)&fr_para;

/**
  * @brief   PID参数恢复出厂配置
  * @param   *pid：pid控制器指针
  * @param   pid_object：PID对象
  */
void _para_restore(struct _pid_para *pid, uint8_t pid_object )
{
    pid->kp = controller_parameter[pid_object][KP];         
    pid->ki = controller_parameter[pid_object][KI];         
    pid->kd = controller_parameter[pid_object][KD];                      
}

/**
  * @brief   PID参数赋值
  * @param   *pid_des：目标pid
  * @param   *pid_src：源pid
  */
void _para_copy(struct _pid_para *pid_des, struct _pid_para *pid_src )
{
    pid_des->kp = pid_src->kp;         
    pid_des->ki = pid_src->ki;         
    pid_des->kd = pid_src->kd;                      
}



