#include "IMU.h"
#include "math.h"
#include "toolbox.h"
// ==================================================================================
// 描述:
// 必须定义'halfT '为周期的一半，以及滤波器的参数Kp和Ki
// 四元数'q0', 'q1', 'q2', 'q3'定义为全局变量
// 需要在每一个采样周期调用'IMUupdate()'函数
// 陀螺仪数据单位是弧度/秒，加速度计的单位无关重要，因为会被规范化
// ==================================================================================
#define Kp 	1.0f    // 比例常数
#define Ki 	0.001f  // 积分常数
#define halfT 0.0005f//半周期
#define T	0.001f  // 周期为1ms

float Yaw,Pitch,Roll;		
// ==================================================================================
// 变量定义
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     	// 四元数
float exInt = 0, eyInt = 0, ezInt = 0;    	// 误差积分累计值 
// ==================================================================================
// 函数原型：void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
// 功        能：互补滤波进行姿态解算
// 输        入：陀螺仪数据及加速度计数据
// ==================================================================================
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
  //四元数积分，求得当前的姿态
	float q0_last = q0;	
	float q1_last = q1;	
	float q2_last = q2;	
	float q3_last = q3;	

	//把加速度计的三维向量转成单位向量
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//估计重力加速度方向在飞行器坐标系中的表示，为四元数表示的旋转矩阵的第三行
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//加速度计读取的方向与重力加速度方向的差值，用向量叉乘计算
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;

	//误差累积，已与积分常数相乘
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;


	//一阶近似算法
	q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
	q1 = q1_last + ( q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
	q2 = q2_last + ( q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
	q3 = q3_last + ( q0_last*gz + q1_last*gy - q2_last*gx)*halfT; 

			
	//四元数规范化
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


