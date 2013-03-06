/*
 * rtc.h
 *
 *  Created on: 03-03-2013
 *      Author: Grzybek
 */

#ifndef RTC_H_
#define RTC_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stm32f10x.h"

#define RTC_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE)

portBASE_TYPE vStartRtcTask(unsigned portBASE_TYPE uxPriority);
portTASK_FUNCTION(RtcTask,parameters);
void RTC_Config();
void RTC_GetTime(uint32_t * thh,uint32_t * tmm, uint32_t * tss);
void RTC_SetTime(uint32_t thh,uint32_t tmm, uint32_t tss);

#endif /* RTC_H_ */
