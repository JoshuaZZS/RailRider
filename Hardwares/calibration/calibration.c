/**
  ******************************************************************************
  * @file    calibration.c
  * @author  Mr.Lin
  * @version V2022.04.01
  * @date    01-Apr-2022
  * @brief   传感器校准
  * @Tips    店铺网站：https://1024tech.taobao.com/
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 源动力科技</center></h2>
  *
  ******************************************************************************
  */

#include "calibration.h"
#include "imath.h"
#include "math.h"
#include "mpu6050.h"
#include "record.h"
#include "flash.h"
#include "queue_msg.h"
#include "stdio.h"
#include "imu.h"

struct CAL_INFO cal_info = {0};
struct SENSOR_CALIBRATE sensor_cal = {0};
const struct SENSOR_CALIBRATE *rsensor_cal = (const struct SENSOR_CALIBRATE *)&sensor_cal;

/*
 * 描述  ：机械中值设置
 * 输入  ：pout_data中值角度数据首地址(float)      
 */
void machineMedian_calibration(uint8_t cmd, float *pin_data , float *pout_data)
{
  if( cal_info.cal_cmd.mechMidan == 1 ) {                 /* 收到设置机械中值操作指令 */
    if( f_abs(*pin_data) < 10.0F ){
      *pout_data = *pin_data;
			switch(cmd){
				case 0xC2:sensor_cal.mechMedian.rol = *pin_data; break;
				case 0xC4:sensor_cal.mechMedian.pit = *pin_data; break;
				default:break;
			}
      uint16_t cal_size = sizeof(sensor_cal) / 4 ;        /* 写入flash时为word单位写入 */
      FLASH_WriteWordData(SENSOR_CAL_ADDRESS , (uint32_t*)&sensor_cal ,cal_size);
			cal_info.cal_cmd.mechMidan = 0;
    }
  }
}
/*
 * 描述  ：三轴零偏数据校准
 * 输入  ：原始的静止时三轴数据首地址(float)      
 */
void zero_calibration(SI_F_XYZ *pin_raw_data , SI_F_XYZ *pout_data )
{
  static uint8_t i = 0;
  static SI_F_XYZ offset = {0.0F};
  static SI_F_XYZ sumoffset = {0.0F};

  if( cal_info.cal_cmd.gyro == 1 ) {                    /* 收到校准操作指令 */
    if( i < CAL_NB ) {                                  /* 累加求平均值 */             
      sumoffset.x += pin_raw_data->x; 
      sumoffset.y += pin_raw_data->y;
      sumoffset.z += pin_raw_data->z;
      i++;
    } 
    else {
      i = 0;
      offset.x = sumoffset.x / CAL_NB;          
      offset.y = sumoffset.y / CAL_NB;          
      offset.z = sumoffset.z / CAL_NB;          
			sif_xyz_memset( &sumoffset , 0.0F);
			dbg("offset x:%.1f	y:%.1f	z:%.1f\r\n",offset.x,offset.y,offset.z);
      if( verify_result( &offset , 600.0F) == 0 ) {     /* 判断校准结果是否合理 */
        sif_xyz_memcopy( pout_data , &offset);  
        sif_xyz_memcopy( &sensor_cal.gyro , &offset);

        uint16_t cal_size = sizeof(sensor_cal) / 4 ;    /* 写入flash时为word单位写入 */
				dbg("cal_size:%d\r\n",cal_size);
        FLASH_WriteWordData(SENSOR_CAL_ADDRESS , (uint32_t*)&sensor_cal ,cal_size);
        cal_info.cal_result.gyro = 1;               
      }
      else {
        sif_xyz_memset( pout_data , 0.0F); 
        cal_info.cal_result.gyro = 0;      
      }
      cal_info.cal_cmd.gyro = 0;
    }
  }
}

/*
 * 函数名：read_cal_para
 * 描述  ：校准数据进行读取
 * 返回  ：err     
 */
int read_cal_para(void)
{
  ErrorStatus err;

	uint16_t sensor_rlens = sizeof(struct SENSOR_CALIBRATE) / 4;
	dbg("len:%d\r\n",sensor_rlens) ;
  FLASH_ReadFloatData(SENSOR_CAL_ADDRESS , (float *)rsensor_cal , sensor_rlens);		/* 每次上电读取出已校准的数据进行使用 */

  dbg("gyro calibration value reading\r\n") ;
  dbg("gyro_offset: %.1f    %.1f    %.1f \n",rsensor_cal->gyro.x , rsensor_cal->gyro.y , rsensor_cal->gyro.z);
  
  /* 陀螺仪校准参数读取 */ 
  if( 0 == isnan(rsensor_cal->gyro.x) && 0 == isnan(rsensor_cal->gyro.y) && 0 == isnan(rsensor_cal->gyro.z) ){
    if( 0 == verify_result( (SI_F_XYZ *)&rsensor_cal->gyro , 600.0F) ){
      sif_xyz_memcopy( &gyro_offset , (SI_F_XYZ *)&rsensor_cal->gyro);     
      err = SUCCESS ;
      dbg("SUCCESS :gyro_offset: %.1f %.1f %.1f \n",gyro_offset.x,gyro_offset.y,gyro_offset.z);
    }
    else{
      sif_xyz_memset( &gyro_offset , 0.0F );
      err = ERROR;
      dbg("ERROR :gyro_offset: %.1f %.1f %.1f \n",gyro_offset.x,gyro_offset.y,gyro_offset.z);
    }    
  }
  else {
    sif_xyz_memset( &gyro_offset , 0.0F );
  }

  /* 机械中值读取 */
	/*rol*/
  if( 0 == isnan(rsensor_cal->mechMedian.rol) ){
    if( f_abs(rsensor_cal->mechMedian.rol) < 10.0F ){
      att.rol_mechineMedian = rsensor_cal->mechMedian.rol;
    }
  }else{
    att.rol_mechineMedian = 0.0F;
  }
	/*pit*/
  if( 0 == isnan(rsensor_cal->mechMedian.pit) ){
    if( f_abs(rsensor_cal->mechMedian.pit) < 10.0F ){
      att.pit_mechineMedian = rsensor_cal->mechMedian.pit;
    }
  }else{
    att.pit_mechineMedian = 0.0F;
  }	
	
  return err;
}

/********************************** 以下为加速度校准 （未使用）**********************************/
/********************************** 以下为加速度校准 （未使用）**********************************/
/********************************** 以下为加速度校准 （未使用）**********************************/

void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ ){
        dS[j] = 0.0f;
        for( k=0; k<6; k++ ){
            JS[j][k] = 0.0f;
        }
    }
}
void Calibrate_Update_Matrices(float dS[6],
                               float JS[6][6],
                               float beta[6],
                               float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    for( j = 0 ; j < 3 ; j++ ) {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }
    for(j = 0 ; j < 6 ; j++) {
        dS[j] += jacobian[j]*residual;
        for(k = 0 ; k < 6 ; k++) {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}
void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;
    //make upper triangular
    for( i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }
    //back-substitute
    for( i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for( j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }
    for( i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}

uint8_t Calibrate_accel(  SI_F_XYZ accel_sample[6], SI_F_XYZ *accel_offsets, SI_F_XYZ *accel_scale)
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3]={0};
    float beta[6]={0};
    float delta[6]={0};
    float ds[6]={0};
    float JS[6][6]={0};
    bool success = true;
    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/gravity_mss;
    while( num_iterations < 20 && change > eps ) {
        num_iterations++;
        Calibrate_Reset_Matrices(ds, JS);

        for( i=0; i<6; i++ ) {
            data[0] = accel_sample[i].x;
            data[1] = accel_sample[i].y;
            data[2] = accel_sample[i].z;
            Calibrate_Update_Matrices(ds, JS, beta, data);
        }
        Calibrate_Find_Delta(ds, JS, delta);
        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);
        for( i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }
    // copy results out
    accel_scale->x = beta[3] * gravity_mss;
    accel_scale->y = beta[4] * gravity_mss;
    accel_scale->z = beta[5] * gravity_mss;
    accel_offsets->x = beta[0] * accel_scale->x;
    accel_offsets->y = beta[1] * accel_scale->y;
    accel_offsets->z = beta[2] * accel_scale->z;

    // sanity check scale
    if(fabsf(accel_scale->x-1.0f) > 0.2f|| fabsf(accel_scale->y-1.0f) > 0.2f|| fabsf(accel_scale->z-1.0f) > 0.2f ) {
        success = false;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if(fabsf(accel_offsets->x) > 3.5f|| fabsf(accel_offsets->y) > 3.5f|| fabsf(accel_offsets->z) > 3.5f ) {
        success = false;
    }
    // return success or failure
    return success;
}
 


/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
