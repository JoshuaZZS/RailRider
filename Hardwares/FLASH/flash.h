/**
  ******************************************************************************
  * @file    flash.h
  * @author  Mr.Lin
  * @version V2022.04.19
  * @date    19-Apr-2022
  * @brief   针对飞控相关的芯片内部flash读写操作函数
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

#define FLASH_SIZE 64                     /* 所选MCU的FLASH容量大小(单位为K) */

#if FLASH_SIZE < 256                      /* flash小于256K字节的芯片的一个扇区地址为1K，否则为2K */
	#define SECTOR_SIZE 1024                /* 字节 */
#else
	#define SECTOR_SIZE 2048                /* 字节 */
#endif

void FLASH_WriteWordData( uint32_t startAddress, uint32_t *writeData, uint16_t countToWrite );
void FLASH_ReadFloatData(uint32_t startAddress,float *readData,uint16_t countToRead);
void FLASH_WriteThreeFloatData( uint32_t startAddress,  float writeData1, float writeData2, float writeData3);

#endif /* __FLASH_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
