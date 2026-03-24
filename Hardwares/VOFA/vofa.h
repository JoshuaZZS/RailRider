/**
  ******************************************************************************
  * @file    vofa.h
  * @author  Mr.Lin
  * @version V2022.04.19
  * @date    19-Apr-2022
  * @brief   与VOFA上位机进行数据通信相关的协议代码及相关功能函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VOFA_H
#define __VOFA_H

#include "stdint.h"

#define ENABLE_VOFA 1

#define CH_COUNT    18
struct Frame {
    float fdata[CH_COUNT];
    unsigned char tail[4] ;
};

void vofa_transform_upload(void);
void vofa_usart_rcvmsg(void);

extern struct Frame vofa_frame;

#endif /* __VOFA_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
