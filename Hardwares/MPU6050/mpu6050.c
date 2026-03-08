/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  Mr.Lin
  * @version V2022.03.31
  * @date    31-Mar-2022
  * @brief   六轴姿态传感器数据读取及相关处理
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "filter.h"
#include "soft_iic.h"
#include "imath.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "imu.h"
#include "queue_msg.h"

/* 模块的A0引脚接GND，IIC的7位地址为0x68，若接到VCC，需要改为0x69 */
#define MPU6050_ADDRESS  (0x68<<1)      //MPU6050器件读地址
#define	mpu_address	    0x68

#define	SMPLRT_DIV		0x19
#define	MPU_CONFIG		0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define FIFO_EN			0x23
#define INT_PIN_CFG		0x37
#define INT_ENABLE		0x38
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define USER_CTRL		  0x6A
#define	PWR_MGMT_1		0x6B
#define PWR_MGMT_2		0x6C
#define	WHO_AM_I		  0x75

S16_XYZ  acc_raw = {0};                       /* 三轴加速度计原始数据存储 */
SI_F_XYZ acc_raw_f = {0.0F};                  /* 三轴加速度计原始数据以浮点类型存储 */
SI_F_XYZ acc_att_lpf = {0.0F};
S16_XYZ  gyro_raw = {0};                      /* 三轴陀螺仪原始数据存储 */
SI_F_XYZ gyro_raw_f = {0.0F};                 /* 三轴陀螺仪原始数据以浮点类型存储 */
SI_F_XYZ gyro_raw_lpf = {0.0F};               /* 三轴陀螺仪原始数据以浮点类型存储(滤波数据储存) */
SI_F_XYZ gyro_raw_cal = {0.0F};               /* 陀螺仪用于校准的原始浮点类型数据存储 */
SI_F_XYZ gyro_offset = {0.0F} ;               /* 陀螺仪零偏数据存储 */
_Mpu6050_data Mpu = {0};                      

/**
  * @brief   写数据到MPU6050寄存器
  * @param   reg_add:寄存器地址
	* @param	 reg_data:要写入的数据
  */
static void MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
  IIC_Write_One_Byte(MPU6050_ADDRESS ,reg_add ,reg_dat);
}

/**
  * @brief   从MPU6050寄存器读取数据
  * @param   reg_add:寄存器地址
	* @param	 Read：存储数据的缓冲区
	* @param	 num：要读取的数据量
  */
static void MPU6050_ReadData(uint8_t reg_add,unsigned char* Read,uint8_t num)
{
  for(uint8_t i = 0; i < num ; i++ ) {
    * ( Read + i ) = IIC_Read_One_Byte(MPU6050_ADDRESS,reg_add);   
    reg_add++;  
  }
}

/**
  * @brief   初始化MOU6050配置
  */
void Mpu6050_init(void)
{  
  delay(10);  
	MPU6050_WriteReg(PWR_MGMT_1, 0x80);	           	
  delay(10);  
	MPU6050_WriteReg(PWR_MGMT_1, 0x00);           /* 唤醒mpu */		
  /* when DLPF is disabled( DLPF_CFG=0 or 7),陀螺仪输出频率 = 8kHz; 
     when DLPFis enabled,陀螺仪输出频率 = 1KHz 
     fs(采样频率) = 陀螺仪输出频率 / (1 + SMPLRT_DIV) */	
	MPU6050_WriteReg(SMPLRT_DIV, 0x00);		        /* sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz */
	MPU6050_WriteReg(MPU_CONFIG, 0x03);           /* 内部低通  acc:44hz	gyro:42hz */
	MPU6050_WriteReg(GYRO_CONFIG, 0x18);			    /* gyro scale  ：+-2000°/s */
	MPU6050_WriteReg(ACCEL_CONFIG, 0x10);			    /* Accel scale ：+-8g (65536/16=4096 LSB/g) */    
  delay(10);  	
}
/**
  * @brief   读取MPU6050的ID
  * @retval  正常返回1，异常返回0
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Ret = 0;
  MPU6050_ReadData( WHO_AM_I , &Ret , 1 );      /* 读器件地址 */
	if(Ret != 0x68) {
		dbg("MPU6050 dectected error! 检测不到MPU6050模块, 请检查模块与开发板的接线! \n");
		return 0;
	}
	else {
		dbg("MPU6050 ID = %d\r\n",Ret);
		return 1;
	}
}

/**
  * @brief   读取MPU6050的三轴加速度原始数据
  * @param   *accRawData：三轴加速度数据指针变量
  */
static void MPU6050ReadAcc(S16_XYZ *accRawData)
{
  uint8_t buf[6];
  MPU6050_ReadData(ACCEL_XOUT_H,buf,6);
  accRawData->x = (buf[0] << 8) | buf[1];
  accRawData->y = (buf[2] << 8) | buf[3];
  accRawData->z = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的三轴角加速度原始数据
  * @param   *gyroRawData：三轴陀螺仪指针变量
  */
static void MPU6050ReadGyro(S16_XYZ *gyroRawData)
{
  uint8_t buf[6];
  MPU6050_ReadData(GYRO_XOUT_H,buf,6);
  gyroRawData->x = (buf[0] << 8) | buf[1];
  gyroRawData->y = (buf[2] << 8) | buf[3];
  gyroRawData->z = (buf[4] << 8) | buf[5];
}
/**
  * @brief   将读取MPU6050的三轴加速度原始数据并转为浮点数据储存
  */
static void get_acc_raw(void)
{
  MPU6050ReadAcc(&acc_raw);
  acc_raw_f.x = (float)acc_raw.x;
  acc_raw_f.y = (float)acc_raw.y;
  acc_raw_f.z = (float)acc_raw.z;
}
/**
  * @brief   巴特沃斯低通滤波器参数（200hz采样率，30hz截止频率）
  */
_Butterworth_parameter gyro_30hz_parameter =
{
  //200hz---30hz
  1,  -0.7477891782585,    0.272214937925,
  0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 
_Butterworth_data   gyro_butter_data[3];

/**
  * @brief   读取陀螺仪三轴原始数据 & 零偏校准去除 & 低通滤波
  */
static void get_gyro_raw(void)
{
  MPU6050ReadGyro(&gyro_raw);
  
  /* 将原始整型数据转为浮点型数据,用于校准时使用 */
  gyro_raw_cal.x = (float)gyro_raw.x;
  gyro_raw_cal.y = (float)gyro_raw.y;
  gyro_raw_cal.z = (float)gyro_raw.z;
  
  /* 原始数据减去校准零偏数据 */
  gyro_raw_f.x = (float)gyro_raw.x - gyro_offset.x;
  gyro_raw_f.y = (float)gyro_raw.y - gyro_offset.y;
  gyro_raw_f.z = (float)gyro_raw.z - gyro_offset.z;     
  
  /* 三轴角速度原始数据进行滤波处理并转为浮点类型 */  
  // gyro_raw_lpf.x = (float)butterworth_lpf( ( (float) gyro_raw_f.x) , &gyro_butter_data[0] , &gyro_30hz_parameter );
  // gyro_raw_lpf.y = (float)butterworth_lpf( ( (float) gyro_raw_f.y) , &gyro_butter_data[1] , &gyro_30hz_parameter );
  // gyro_raw_lpf.z = (float)butterworth_lpf( ( (float) gyro_raw_f.z) , &gyro_butter_data[2] , &gyro_30hz_parameter );
  
  /* 三轴角速度原始数据进行滤波处理并转为浮点类型 */
  butterworth_lpf_run(&gyro_raw_f , &gyro_raw_lpf , gyro_30hz_parameter);   /* 此处新增，封装成三轴滤波函数 */
}
/**
  * @brief   （滤波后）原始加速度转为重力加速度g为单位数据
  * @param   *acc_in：原始加速度浮点类型指针变量
  * @param   *acc_out：以g为单位的加速度浮点数据指针变量
  */
static void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
	acc_out->x = (float)(acc_in->x * acc_raw_to_g);
	acc_out->y = (float)(acc_in->y * acc_raw_to_g);
	acc_out->z = (float)(acc_in->z * acc_raw_to_g);
}
/**
  * @brief   （滤波后）原始陀螺仪数据转为弧度/秒为单位的数据
  * @param   *gyro_in：原始的陀螺仪浮点数据指针变量
  * @param   *gyro_out：以rad/s为单位的陀螺仪浮点数据指针变量
  */
static void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out)
{
	gyro_out->x = (float)(gyro_in->x * gyro_raw_to_radian_s);
	gyro_out->y = (float)(gyro_in->y * gyro_raw_to_radian_s);
	gyro_out->z = (float)(gyro_in->z * gyro_raw_to_radian_s);
}
/**
  * @brief   （滤波后）原始陀螺仪数据转为度/秒为单位的数据
  * @param   *gyro_in：原始的陀螺仪浮点数据指针变量
  * @param   *gyro_deg_out：以deg/s为单位的陀螺仪浮点数据指针变量
  */
static void get_deg_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_deg_out)
{
	gyro_deg_out->x = (float)(gyro_in->x * gyro_raw_to_deg_s);
	gyro_deg_out->y = (float)(gyro_in->y * gyro_raw_to_deg_s);
	gyro_deg_out->z = (float)(gyro_in->z * gyro_raw_to_deg_s);    
}

/**
  * @brief   mpu6050功能函数运行
  */
void mpu6050_loop(void)
{
  get_acc_raw();                                                //读取加速度原始数据
  get_gyro_raw();                                               //读取陀螺仪原始数据
  get_deg_s(&gyro_raw_lpf,&Mpu.deg_s);                            //陀螺仪原始数据转为度为单位的速率    
  get_rad_s(&gyro_raw_lpf,&Mpu.rad_s);                            //陀螺仪原始数据转为弧度为单位的速率
  get_acc_raw();                                                //加速度数据
  iir_lpf_run(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);      //姿态解算时加速度低通滤波 
  get_acc_g(&acc_att_lpf,&Mpu.acc_g);                           //将加速度数据转为以g为单位的加速度数据
  mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z); 
  Matrix_ready();                                               //姿态解算相关矩阵更新
}

/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
