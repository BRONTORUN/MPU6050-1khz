#include "IMU.h"
#include "math.h"
#include "toolbox.h"
// ==================================================================================
// ����:
// ���붨��'halfT 'Ϊ���ڵ�һ�룬�Լ��˲����Ĳ���Kp��Ki
// ��Ԫ��'q0', 'q1', 'q2', 'q3'����Ϊȫ�ֱ���
// ��Ҫ��ÿһ���������ڵ���'IMUupdate()'����
// ���������ݵ�λ�ǻ���/�룬���ٶȼƵĵ�λ�޹���Ҫ����Ϊ�ᱻ�淶��
// ==================================================================================
#define Kp 	1.0f    // ��������
#define Ki 	0.001f  // ���ֳ���
#define halfT 0.0005f//������
#define T	0.001f  // ����Ϊ1ms

float Yaw,Pitch,Roll;		
// ==================================================================================
// ��������
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     	// ��Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;    	// �������ۼ�ֵ 
// ==================================================================================
// ����ԭ�ͣ�void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
// ��        �ܣ������˲�������̬����
// ��        �룺���������ݼ����ٶȼ�����
// ==================================================================================
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
  //��Ԫ�����֣���õ�ǰ����̬
	float q0_last = q0;	
	float q1_last = q1;	
	float q2_last = q2;	
	float q3_last = q3;	

	//�Ѽ��ٶȼƵ���ά����ת�ɵ�λ����
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//�����������ٶȷ����ڷ���������ϵ�еı�ʾ��Ϊ��Ԫ����ʾ����ת����ĵ�����
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//���ٶȼƶ�ȡ�ķ������������ٶȷ���Ĳ�ֵ����������˼���
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;

	//����ۻ���������ֳ������
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//�ò���������PI����������ƫ�����������ݶ����е�ƫ����	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;


	//һ�׽����㷨
	q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
	q1 = q1_last + ( q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
	q2 = q2_last + ( q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
	q3 = q3_last + ( q0_last*gz + q1_last*gy - q2_last*gx)*halfT; 

			
	//��Ԫ���淶��
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	Roll = atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3)*Radian_to_Angle;
	Pitch = asin (2.0f*(q0*q2 - q1*q3))*Radian_to_Angle;
  Yaw = atan2(2.0f*(q1*q2 + q0*q3),q0*q0 + q1*q1 - q2*q2 - q3*q3)*Radian_to_Angle;
	//Yaw = Yaw + gyro.z*RawData_to_Angle*0.001;
	
	/*printf("q0 %f  ",q0);
	printf("q1 %f  ",q1);
	printf("q2 %f  ",q2);
	printf("q3 %f  ",q3);*/
	
	/*printf("Roll %f  ",Roll);
	printf("Pitch %f  ",Pitch);
	printf("Yaw %f  \n",Yaw);*/
}


