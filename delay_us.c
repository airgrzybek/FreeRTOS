/*
 * delay_us.c
 *
 *  Created on: 03-02-2013
 *      Author: Grzybek
 */

#include "stm32f10x.h"
#include "delay_us.h"

#define SYS_CLK 72000000
#define TIMER_FREQ 10000000

void delay_us_init(void)
{
    /* Enable timer clock  - use TIMER5 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);


    /* Time base configuration */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = (SYS_CLK / TIMER_FREQ) - 1;
    TIM_TimeBaseStructure.TIM_Period = UINT16_MAX;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

   /* Enable counter */
    TIM_Cmd(TIM5, ENABLE);
}

void delay_us(u16 us)
{
    TIM5->CNT = 0;
    /* use 16 bit count wrap around */
    while((uint16_t)(TIM5->CNT) <= us);
}
