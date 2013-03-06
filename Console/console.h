/*
 * console.h
 *
 *  Created on: 02-02-2013
 *      Author: Grzybek
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

/* Standard includes. */
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+IO includes. */
#include "FreeRTOS_IO.h"

/* Example includes. */
#include "FreeRTOS_CLI.h"
#include "console.h"
#include "CLI-commands.h"

#define CONSOLE_USART                       "/USART1/"
#define CONSOLE_BAUD_RATE                   115200
#define CONSOLE_INTERRUPT_PRIORITY          5
#define CONSOLE_STACK_SIZE                  configMINIMAL_STACK_SIZE + 1000
#define CONSOLE_TASK_PRIORITY               tskIDLE_PRIORITY
#define CONSOLE_CMD_MAX_OUTPUT_SIZE         100
#define CONSOLE_CMD_MAX_INPUT_SIZE          50


portBASE_TYPE vStartConsoleTask(void);
portTASK_FUNCTION(ConsoleTask,parameters);
Peripheral_Descriptor_t * GetConsoleDescriptor(void);

void putChar(char c);

#endif /* CONSOLE_H_ */
