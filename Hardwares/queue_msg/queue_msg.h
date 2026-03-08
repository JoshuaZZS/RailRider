/**
  ******************************************************************************
  * @file    queue_msg.h
  * @author  Mr.Lin
  * @version V2.10.18
  * @date    11-Mar-2022
  * @brief   队列处理
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QUEUE_MSG_H
#define __QUEUE_MSG_H

#include "stdint.h"


void uart_print(const char *format, ...);
void uart_print_array(char*name , uint8_t *buf, uint16_t len , uint8_t mode);

#define DEBUG_ENABLE 0

#if (1 == DEBUG_ENABLE)
#define dbg  uart_print
#define print_arry(name, buf, len , mode)        uart_print_array(name, buf, len , mode)
#else
#define dbg(...)          do{}while(0)
#define print_arry(...)   do{}while(0)
#endif

#define USART_BUFFSIZE    256   // 定义缓冲区的大小
#define USART_NAMESIZE    10 
typedef struct {
    uint16_t SendLens;    //待发送数据长度
    uint16_t RecvLens;    //接收到的数据长度
    uint16_t RecvQue_Head;      //新接收数据环形队列头指针
    uint16_t RecvQue_Tail;      //新接收数据环形队列尾指针
    uint8_t SentBuff[USART_BUFFSIZE];
    uint8_t RecvBuff[USART_BUFFSIZE];
    uint8_t RecvFlags;
} USART_STR;

typedef struct {
    uint8_t uart_rxbuf[USART_BUFFSIZE];
    uint8_t uart_rxname[USART_NAMESIZE];
} UART_MessageQueue_TypeDef;

uint16_t Usart3_DMA_Sent(uint8_t * Sendbuff, uint16_t Bufflens);
void pu8printf(char* name, uint8_t* pbuf, uint32_t count, uint8_t Mode);
void Usart_RecvData(void);

extern USART_STR Usart3_Str ;
extern USART_STR Usart2_Str ;

#endif /* __QUEUE_MSG_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
