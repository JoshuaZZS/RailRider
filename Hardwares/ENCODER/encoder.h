#ifndef _ENCODER_h_
#define _ENCODER_h_


#include "stdint.h"
#include "main.h"

#define FULL_ENCODER 50000


typedef struct 
{
  int nb1_val;
  int nb2_val;
  int nb3_val;
  uint8_t dir1; 
  uint8_t dir2;
  uint8_t dir3; 
  
}_ENCODER_INFO;

uint8_t readEncoderValue(void);

extern _ENCODER_INFO encoderINFO ;

#endif
