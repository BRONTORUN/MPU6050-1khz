#ifndef __STRUCT_H
#define __STRUCT_H
#include "stm32f10x.h"   // 相当于51单片机中的  #include <reg51.h>
/******************************************************************************
							结构体声明
*******************************************************************************/ 
/* MPU6050--加速度计结构体 */
struct _acc
{
	int16_t x;
	int16_t y;
	int16_t z;
};

/* MPU6050--陀螺仪结构体 */
struct _gyro
{
	int16_t x;
	int16_t y;
	int16_t z;
};
/* float结构体 */
struct _SI_float
{
	float x;
	float y;
	float z;
};

extern struct _acc  acc;			//原始数据
extern struct _gyro gyro;
////////////////////////////////////////////
extern struct _acc  offset_acc;	//零偏数据
extern struct _gyro offset_gyro;
////////////////////////////////////////////
extern struct _SI_float gyro_float;//浮点数据

#endif
