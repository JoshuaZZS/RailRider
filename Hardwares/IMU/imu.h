#ifndef _IMU_H_
#define _IMU_H_


typedef struct 
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float pit;
  float rol;
  float yaw;
  float pit_mechineMedian;
  float rol_mechineMedian;
  float yaw_mechineMedian;
}_Attitude;

typedef struct
{
  float DCM[3][3];        //机体坐标系 -> 地理坐标系
  float DCM_T[3][3];      //地理坐标系 -> 机体坐标系
}_Matrix;

extern _Matrix Mat;
extern _Attitude att;

void mahony_update(float gx, float gy, float gz, float ax, float ay, float az) ;
void rotation_matrix(void);
void rotation_matrix_T(void);
void Matrix_ready(void);








#endif

