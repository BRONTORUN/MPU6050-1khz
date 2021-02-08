#include "stm32f10x.h" 
#include "tim.h"
uint32_t Timer3_Count = 0;//��¼Timer3�жϴ���
uint16_t Timer3_Frequency;//Timer3�ж�Ƶ��
uint16_t Count_1ms,Count_2ms,Count_4ms;
/******************************************************************************
����ԭ�ͣ�	void Timer3_Init(uint16_t Handler_Frequency)
��    �ܣ�	��ʼ����ʱ��3
��    ����   Handler_FrequencyΪTimer3�ж�Ƶ��
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	
	TIM_TimeBaseStructure.TIM_Period = (1000*1000/Handler_Frequency) ;//װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	//��Ƶϵ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //���ָ�ʱ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//����жϱ�־
	
	//Timer3
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM3,ENABLE);//ʹ�ܶ�ʱ��3
}
void TIM3_IRQHandler(void)//Timer3�ж�
{	
	/*if(TIM3->SR & TIM_IT_Update)
	{     
		TIM3->SR = ~TIM_FLAG_Update;//����жϱ�־
		Count_1ms++;
		
	}*/
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{
		Count_1ms++;
		Count_2ms++;
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}