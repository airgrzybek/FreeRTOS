/*
 * TouchTask.h
 *
 *  Created on: 12-02-2013
 *      Author: Grzybek
 */

#ifndef TOUCHTASK_H_XYZ
#define TOUCHTASK_H_XYZ

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "touch_driver.h"

#define TOUCH_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE + 100)

#define SAMPLE_RATE                 10


portBASE_TYPE vStartTouchTask(unsigned portBASE_TYPE uxPriority);
portTASK_FUNCTION(TouchTask,parameters);

TouchStatus getTouch(u16 * xAxis, u16 * yAxis);


#endif /* TOUCHTASK_H_ */
