#include "stm32f10x.h"   // 相当于51单片机中的  #include <reg51.h>
#include "i2c.h"

float invSqrt(float x);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) ;