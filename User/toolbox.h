#include "stm32f10x.h"   // 相当于51单片机中的  #include <reg51.h>
#include "struct.h"
#define RawData_to_Radian	0.0010653f
#define RawData_to_Angle	0.0610351f	//以下参数对应2000度每秒
#define Radian_to_Angle	   57.2957795f
float invSqrt(float x);
void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out);	
