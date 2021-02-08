#include "stm32f10x.h"   // �൱��51��Ƭ���е�  #include <reg51.h>
#include "usart.h"
#include "i2c.h"
#include "IMU.h"
#include "tim.h"
#include "struct.h"
#include "toolbox.h"
int16_t err_cnt=0;
int main(void)
{
	// ���������ʱ��ϵͳ��ʱ���Ѿ������ó�72M��
	
	Uart1_Init(115200);
	i2c_init();
	InitMPU6050();
	Timer3_Init(1000);	//Timer3�ж�1KHZ
	while(1)
	{
		if(Count_1ms >= 1)//1000hzƵ�ʻ�
		{
			Count_1ms = 0;
			if(!MPU6050_SequenceRead())
			{
				err_cnt++;
			}
			if(ACC_Offset==1)
			{
				MPU6050_Compose();
				continue;
			}
			MPU6050_Compose();
			Get_Radian(&gyro,&gyro_float);
			IMUupdate(gyro_float.x,gyro_float.y,gyro_float.z,acc.x,acc.y,acc.z);
			/*printf("err_cnt = %d ",err_cnt);
			printf("acc.x = %d ",acc.x);
			printf("acc.y = %d ",acc.y);
			printf("acc.z = %d ",acc.z);
			printf("gyro.x = %d ",gyro.x);
			printf("gyro.y = %d ",gyro.y);
			printf("gyro.z = %d\n",gyro.z);*/
			
		}
	}
}
