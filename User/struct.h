#ifndef __STRUCT_H
#define __STRUCT_H
#include "stm32f10x.h"   // �൱��51��Ƭ���е�  #include <reg51.h>
/******************************************************************************
							�ṹ������
*******************************************************************************/ 
/* MPU6050--���ٶȼƽṹ�� */
struct _acc
{
	int16_t x;
	int16_t y;
	int16_t z;
};

/* MPU6050--�����ǽṹ�� */
struct _gyro
{
	int16_t x;
	int16_t y;
	int16_t z;
};
/* float�ṹ�� */
struct _SI_float
{
	float x;
	float y;
	float z;
};

extern struct _acc  acc;			//ԭʼ����
extern struct _gyro gyro;
////////////////////////////////////////////
extern struct _acc  offset_acc;	//��ƫ����
extern struct _gyro offset_gyro;
////////////////////////////////////////////
extern struct _SI_float gyro_float;//��������

#endif
