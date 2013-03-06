/*
 * CLI-commands.h
 *
 *  Created on: 02-02-2013
 *      Author: Grzybek
 */

#ifndef CLI_COMMANDS_H_
#define CLI_COMMANDS_H_

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

/* board specific includes */
#include "LcdTask.h"
#include "LcdMessage.h"

void vRegisterCLICommands(void);
/*
 * Implements the dummy command.
 */portBASE_TYPE prvDummyCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString);

portBASE_TYPE prvLcdCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString);

portBASE_TYPE prvMemAccessCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString);

portBASE_TYPE prvRunTimeStatsCommand(int8_t *pcWriteBuffer,
        size_t xWriteBufferLen, const int8_t *pcCommandString);

portBASE_TYPE prvTaskStatsCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString);

portBASE_TYPE prvSDIOCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString);

portBASE_TYPE prvTimeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
        const int8_t *pcCommandString);

#endif /* CLI_COMMANDS_H_ */
