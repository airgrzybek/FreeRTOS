/*
 * LcdTask.h
 *
 *  Created on: 29-01-2013
 *      Author: Grzybek
 */

#ifndef LCDTASK_H_
#define LCDTASK_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "lcd.h"
#include "LcdMessage.h"

#define MESSAGES_QUEUE_LENGTH       10
#define LCD_TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE + 2000)
#define NO_BLOCK                    ((portTickType) 0)

portBASE_TYPE vStartLcdTask(unsigned portBASE_TYPE uxPriority);
portTASK_FUNCTION(LcdTask,parameters);
portBASE_TYPE LCD_SendCmd(LcdCmdMsg * msg);
portBASE_TYPE LCD_Cmd(LcdCmd cmd, ...);

#endif /* LCDTASK_H_ */
