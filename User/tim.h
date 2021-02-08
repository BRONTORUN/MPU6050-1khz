#ifndef _TIMER3_H_
#define _TIMER3_H_
#include "stm32f10x.h"

/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint32_t Timer3_Count;
extern uint16_t Timer3_Frequency;
extern uint16_t Count_1ms,Count_2ms,Count_4ms;
/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency);
void TIM3_IRQHandler(void);//Timer3中断
#endif