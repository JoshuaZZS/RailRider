#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "stdint.h"

#define JUMP_ENABLE 0

/* 以电机轴朝向自己本人为基准(飞轮朝向自己为准) */
#define dir1Clockwise()      HAL_GPIO_WritePin(MOTOR_DIR1_GPIO_Port, MOTOR_DIR1_Pin, GPIO_PIN_SET)   //顺时针方向
#define dir1Anticlockwise()  HAL_GPIO_WritePin(MOTOR_DIR1_GPIO_Port, MOTOR_DIR1_Pin, GPIO_PIN_RESET) //逆时针方向
#define Motor1EN()       HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET)
#define Motor1Stop()     HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_RESET)

#define dir2Clockwise()      HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_SET)   //顺时针方向
#define dir2Anticlockwise()  HAL_GPIO_WritePin(MOTOR_DIR2_GPIO_Port, MOTOR_DIR2_Pin, GPIO_PIN_RESET) //逆时针方向
#define Motor2EN()       HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET)
#define Motor2Stop()     HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_RESET)

#define AIN1_HIGH   HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET)
#define AIN1_LOW    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET)
#define AIN2_HIGH   HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET)
#define AIN2_LOW    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET)
#define STBY_HIGH   HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET)
#define STBY_LOW    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET)


typedef struct 
{
  int out;
}_OUT_Motor;

typedef struct 
{
  uint16_t tick;
  uint8_t flag;
  uint8_t dir;
  uint16_t cnt;
}_JUMP_INFO;


/* 小车平衡控制 begin*/
typedef struct
{
    int balance;
    int velocity;
    int turn;
    
    float bais[3];
    float exp[3];
    
    int pwm[3];
    int out[3];
    int motor[2];
    
}_OUT_CONTROL;

typedef struct
{
    float kp;
    float ki;
    float kd;
}_PID_CONTROL;

/* end */

void _controller_perform(void) ;
void _controller_output(void)   ;

extern _OUT_Motor motor1;
extern _OUT_Motor motor2;
extern _OUT_Motor motor3;

extern _PID_CONTROL vel;
extern _PID_CONTROL bal;
#endif
