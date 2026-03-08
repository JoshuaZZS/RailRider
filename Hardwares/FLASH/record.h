/**
  ******************************************************************************
  * @file    record.h
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

#ifndef __RECORD_H
#define __RECORD_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "flash.h"

#define START_ADDRESS           (FLASH_BASE + (SECTOR_SIZE*50))
#define FLASH_USER_START_ADDR   START_ADDRESS                       /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (START_ADDRESS + FLASH_PAGE_SIZE)   /* End @ of user Flash area */

#define SENSOR_CAL_ADDRESS    START_ADDRESS + 36
#define PID_GROUP_ADDRESS    (START_ADDRESS + SECTOR_SIZE)

#define RECORD_MAGIC   0x4b544454U

enum _pid_type_nb
{	
  ANGLE_PIT = 0,        
  ANGLE_ROL,
  ANGLE_YAW,
  GYRO_PIT,
  GYRO_ROL,
  GYRO_YAW
};

enum _pid_para_nb 
{	
  KP = 0,  
  KI,       
  KD,  
};
struct _pid_para
{
    float kp;
    float ki;
    float kd;
};

struct _pid_class
{
    struct _pid_para pit;
    struct _pid_para rol;
    struct _pid_para yaw;
};

struct _para_group
{
    uint32_t magic;   
    struct _pid_class angle;
    struct _pid_class gyro;
};

void _para_restore(struct _pid_para *pid, uint8_t pid_object );
void _para_copy(struct _pid_para *pid_des, struct _pid_para *pid_src );

extern struct _para_group fr_para;
extern struct _para_group *wfr_para;
extern const struct _para_group *rfr_para;
#endif
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
