/**
  ******************************************************************************
  * @file    queue_msg.c
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
#include "queue_msg.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "ano.h"
#include <stdarg.h>
#include "cmsis_os.h"
#include "queue.h"

USART_STR Usart3_Str;
USART_STR Usart2_Str ;
static char log_buf[512] = { 0 };
extern osMessageQId QueueUart3Handle;
extern osMessageQId QueueUart2Handle;
extern DMA_HandleTypeDef hdma_usart1_tx;

#pragma import(__use_no_semihosting)             
               
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
   
void _sys_exit(int x) 
{ 
	x = x; 
}
   
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);	/* 发送一个字节数据到串口DEBUG_USART */
	return (ch);
}


/**
 * This function is print flash non-package info.
 *
 * @param format output format
 * @param ... args
 */
void uart_print(const char *format, ...) {
    va_list args;
    /* args point to the first variable parameter */
    va_start(args, format);
    /* must use vprintf to print */
    vsprintf(log_buf, format, args);
    printf("%s", log_buf);
    va_end(args);
}

/**
  * @brief   数组打印
  * @param   name ：数组名称字符串
  * @param   buf ：数据内容首地址
  * @param   len ：数据长度
  * @param   mode: 打印方式
  */
void uart_print_array(char*name , uint8_t *buf, uint16_t len , uint8_t mode)
{
	uint16_t i = 0;
  uart_print("%s:\r\n", name);   
	for(i = 0; i < len; i++) {
    if( 1 == mode ){
      uart_print("%d ", buf[i]);
    }
    else {
      uart_print("%02x ", buf[i]);
    }
		
		if( (i != 0) && (0 == i%32) ) {
			uart_print("\r\n");
		}
	}
  uart_print("\r\n");
}


/**
  * @brief   串口1DMA数据发送
  * @param   Sendbuff ：缓冲数据首地址
  * @param   Bufflens ：数据长度
  * @retval  l_val：数据长度
  */
uint16_t Usart3_DMA_Sent(uint8_t * Sendbuff, uint16_t Bufflens)
{
	uint16_t l_val = Bufflens > USART_BUFFSIZE ? USART_BUFFSIZE : Bufflens;
	if(Bufflens <= 0) {
		return 0;
	}
	if(Sendbuff) {
		memcpy( (void*) Usart3_Str.SentBuff, (const void*)Sendbuff, l_val);
	}
	HAL_UART_Transmit_DMA(&huart3, Usart3_Str.SentBuff, l_val);
	
  return l_val;
}

/**
  * @brief   串口1接收数据处理函数
  */


void Usart_RecvData(void)
{
	static UART_MessageQueue_TypeDef rmsg ;
	if( Usart3_Str.RecvFlags == 1 ) { 
		dbg("\nrlens:%d\r\n", Usart3_Str.RecvLens);
		print_arry("rbuf",Usart3_Str.RecvBuff,Usart3_Str.RecvLens,0);
		ANO_DT_Data_Receive_Anl(Usart3_Str.RecvBuff ,Usart3_Str.RecvLens);   //进行数据解析
		memset (Usart3_Str.RecvBuff , 0,Usart3_Str.RecvLens);                 //缓存数据清除
		memset (rmsg.uart_rxbuf , 0,USART_BUFFSIZE);                 //缓存数据清除
		Usart3_Str.RecvLens = 0;                                              //接收的数据长度清除
		Usart3_Str.RecvFlags = 0;                                             //标志位清除
	}    
}




/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
