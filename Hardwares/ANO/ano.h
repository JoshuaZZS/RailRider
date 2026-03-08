/**
  ******************************************************************************
  * @file    ano.h
  * @author  Mr.Lin
  * @version V2022.04.19
  * @date    19-Apr-2022
  * @brief   与匿名地面站上位机进行数据通信相关的协议代码及相关功能函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANO_H
#define __ANO_H

#include "stdint.h"

#define SENSOR_GYRO 02
#define USART_RX_LEN 100 

#define CAL_SUCCESS             0x01
#define CAL_FAIL                0xE1
#define SETUP_SUCCESS           0x31
#define RESTORE_DEFAULT_SUCCESS 0xA1

struct send_bits
{
  uint16_t version:1;
  uint16_t status:1;
  uint16_t senser:1;
  uint16_t pid1:1;
  uint16_t pid2:1;
  uint16_t pid3:1;
  uint16_t pid4:1;
  uint16_t pid5:1;
  uint16_t pid6:1;
  uint16_t rcdata:1;
  uint16_t offset:1;
  uint16_t motopwm:1;
  uint16_t power:1;
  uint16_t reserved:3;
};

union dt_flag_t
{
  uint16_t reg;
  struct send_bits bit;
};


void ANO_DMA_SEND_DATA(void);
void ANO_DT_Data_Receive_Prepare(uint8_t data);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);

#endif /* __ANO_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
