/*
 * ADC_Task.h
 *
 *  Created on: 03-03-2013
 *      Author: Grzybek
 */

#ifndef ADC_TASK_H_
#define ADC_TASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stm32f10x.h"

#define ADC_STACK_SIZE          (configMINIMAL_STACK_SIZE + 100)


portBASE_TYPE vStartAdcTask(unsigned portBASE_TYPE uxPriority);
portTASK_FUNCTION(AdcTask,parameters);
void AdcConfig(void);

#endif /* ADC_TASK_H_ */
