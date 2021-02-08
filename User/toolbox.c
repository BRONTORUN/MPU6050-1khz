#include "toolbox.h"
#include "struct.h"

// ���ټ��㿪���ŵĵ���
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/******************************************************************************
����ԭ�ͣ�	void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
��    �ܣ�	���ٶ���ԭʼ����תΪ����
*******************************************************************************/ 
void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
{
	Gyro_out->x = (float)(Gyro_in->x * RawData_to_Radian);
	Gyro_out->y = (float)(Gyro_in->y * RawData_to_Radian);
	Gyro_out->z = (float)(Gyro_in->z * RawData_to_Radian);
}
