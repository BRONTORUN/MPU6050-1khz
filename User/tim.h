#ifndef _TIMER3_H_
#define _TIMER3_H_
#include "stm32f10x.h"

/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 
extern uint32_t Timer3_Count;
extern uint16_t Timer3_Frequency;
extern uint16_t Count_1ms,Count_2ms,Count_4ms;
/******************************************************************************
							ȫ�ֺ�������
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency);
void TIM3_IRQHandler(void);//Timer3�ж�
#endif