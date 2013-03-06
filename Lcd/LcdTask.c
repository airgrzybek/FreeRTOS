/*
 * LcdTask.c
 *
 *  Created on: 29-01-2013
 *      Author: Grzybek
 */

#include "LcdTask.h"
#include <stdarg.h>

static xQueueHandle xCmdMessages;

portBASE_TYPE vStartLcdTask(unsigned portBASE_TYPE uxPriority)
{
    portBASE_TYPE result = pdTRUE;
    xCmdMessages = xQueueCreate(MESSAGES_QUEUE_LENGTH,sizeof(LcdCmdMsg));

    if (0 == xCmdMessages)
    {
        // If queue was not created we return
        //LCD_DisplayStringLine(Line0, "xMessages queue error!");
        result = pdFALSE;
    }
    else
    {

        // Create a task
        if(pdPASS == xTaskCreate( LcdTask, ( signed portCHAR * ) "LCD", LCD_TASK_STACK_SIZE,
                NULL, uxPriority, NULL))
        {
            result = pdTRUE;
        }
        else
        {
            result = pdFALSE;
        }
    }

    return result;
}

portTASK_FUNCTION(LcdTask,parameters)
{
    LcdCmdMsg lcdCmd;
    LcdCmd CMD;

    portENTER_CRITICAL();
    // configure lcd
    STM3210E_LCD_Init();
    LCD_Clear(White);
    LCD_SetBackColor(White);
    LCD_SetTextColor(Blue);
    LCD_DisplayStringLine(Line0, "LCD TASK 2 READY!");
    portEXIT_CRITICAL();

    while (1)
    {
        while (pdTRUE != xQueueReceive(xCmdMessages,&lcdCmd,portMAX_DELAY));

        //CMD = lcdCmd.CMD;

        switch (lcdCmd.CMD)
        {
        case DISPLAY_LINE:
            //portENTER_CRITICAL();
            LCD_ClearLine(
                    ((LcdDisplayLineCmd*) lcdCmd.params)->line * LCD_LINE_SIZE);
            LCD_DisplayStringLine(
                    ((LcdDisplayLineCmd*) lcdCmd.params)->line * LCD_LINE_SIZE,
                    ((LcdDisplayLineCmd*) lcdCmd.params)->message);
            //portEXIT_CRITICAL();
            free(lcdCmd.params);
            lcdCmd.params = NULL;
            break;

        case DRAW_CIRCLE:
            //portENTER_CRITICAL();
            LCD_DrawCircle(((LcdDrawCircleCmd*) lcdCmd.params)->x,
                    ((LcdDrawCircleCmd*) lcdCmd.params)->y,
                    ((LcdDrawCircleCmd*) lcdCmd.params)->rad);
            //portEXIT_CRITICAL();
            free(lcdCmd.params);
            lcdCmd.params = NULL;
            break;
        case SET_BACK_COLOR:
            //portENTER_CRITICAL();
            LCD_SetBackColor(((LcdSetColorCmd*)lcdCmd.params)->color);
            //portEXIT_CRITICAL();
            free(lcdCmd.params);
            lcdCmd.params = NULL;
            break;
        case SET_TEXT_COLOR:
            //portENTER_CRITICAL();
            LCD_SetTextColor(((LcdSetColorCmd*)lcdCmd.params)->color);
            //portEXIT_CRITICAL();
            free(lcdCmd.params);
            lcdCmd.params = NULL;
            break;
        case CLEAR:
            //taskENTER_CRITICAL();
            LCD_Clear(((LcdSetColorCmd*)lcdCmd.params)->color);
            //taskEXIT_CRITICAL();
            free(lcdCmd.params);
            lcdCmd.params = NULL;
            break;
        case DISPLAY_LINE_COLUMN:
            LCD_DisplayStringLineColumn(
                    ((LcdDisplayLineColumnCmd*) lcdCmd.params)->line * LCD_LINE_SIZE
                            ,((LcdDisplayLineColumnCmd*) lcdCmd.params)->column,
                    ((LcdDisplayLineColumnCmd*) lcdCmd.params)->message,
                    ((LcdDisplayLineColumnCmd*) lcdCmd.params)->size);
            //portEXIT_CRITICAL();
            free(lcdCmd.params);
            lcdCmd.params = NULL;
            break;
        default:
            break;

        }

        /*
         * Free memory after message
         */


        /*        portENTER_CRITICAL();
         LCD_ClearLine(lcdMessage.line*LCD_LINE_SIZE);
         LCD_DisplayStringLine(lcdMessage.line*LCD_LINE_SIZE + lcdMessage.column,
         lcdMessage.message);
         portEXIT_CRITICAL();*/

        vTaskDelay(5 / portTICK_RATE_MS);

    }

}


portBASE_TYPE LCD_SendCmd(LcdCmdMsg * msg)
{
    if(pdFALSE == xQueueIsQueueFullFromISR(xCmdMessages))
    {
        return xQueueSendFromISR(xCmdMessages,msg,portMAX_DELAY);
    }
    else
    {
        return pdFALSE;
    }
}

portBASE_TYPE LCD_Cmd(LcdCmd cmd, ...)
{
    va_list argList;
    LcdCmdMsg cmdMsg;
    portBASE_TYPE xReturn = pdPASS;
    void * payload = NULL;

    va_start(argList, cmd);

    switch (cmd)
    {
    case DISPLAY_LINE:
        payload = (LcdDisplayLineCmd*) malloc(sizeof(LcdDisplayLineCmd));
        ((LcdDisplayLineCmd*) payload)->message = va_arg(argList,char*);
        ((LcdDisplayLineCmd*) payload)->line = va_arg(argList,int);
        cmdMsg.CMD = DISPLAY_LINE;
        cmdMsg.params = (void*) payload;
        xReturn = LCD_SendCmd(&cmdMsg);
        break;
    case DISPLAY_LINE_COLUMN:
        payload = (LcdDisplayLineColumnCmd*) malloc(sizeof(LcdDisplayLineColumnCmd));
        ((LcdDisplayLineColumnCmd*) payload)->message = va_arg(argList,char*);
        ((LcdDisplayLineColumnCmd*) payload)->line = va_arg(argList,int);
        ((LcdDisplayLineColumnCmd*) payload)->column = va_arg(argList,int);
        ((LcdDisplayLineColumnCmd*) payload)->size = va_arg(argList,int);
        cmdMsg.CMD = DISPLAY_LINE_COLUMN;
        cmdMsg.params = (void*) payload;
        xReturn = LCD_SendCmd(&cmdMsg);
        break;

    case DRAW_CIRCLE:
        payload = (LcdDrawCircleCmd*) malloc(sizeof(LcdDrawCircleCmd));
        ((LcdDrawCircleCmd*) payload)->x = (u8)va_arg(argList,int);
        ((LcdDrawCircleCmd*) payload)->y = (u16)va_arg(argList,int);
        ((LcdDrawCircleCmd*) payload)->rad = (u16)va_arg(argList,int);
        cmdMsg.CMD = DRAW_CIRCLE;
        cmdMsg.params = (void*) payload;
        xReturn = LCD_SendCmd(&cmdMsg);
        break;
    case SET_TEXT_COLOR:
        payload = (LcdSetColorCmd*) malloc(sizeof(LcdSetColorCmd));
        ((LcdSetColorCmd*) payload)->color = (u16)va_arg(argList,int);
        cmdMsg.CMD = SET_TEXT_COLOR;
        cmdMsg.params = (void*) payload;
        xReturn = LCD_SendCmd(&cmdMsg);
        break;
    case SET_BACK_COLOR:
        payload = (LcdSetColorCmd*) malloc(sizeof(LcdSetColorCmd));
        ((LcdSetColorCmd*) payload)->color = (u16)va_arg(argList,int);
        cmdMsg.CMD = SET_BACK_COLOR;
        cmdMsg.params = (void*) payload;
        xReturn = LCD_SendCmd(&cmdMsg);
        break;
    case CLEAR:
        payload = (LcdSetColorCmd*) malloc(sizeof(LcdSetColorCmd));
        ((LcdSetColorCmd*) payload)->color = (u16) va_arg(argList,int);
        cmdMsg.CMD = CLEAR;
        cmdMsg.params = (void*) payload;
        xReturn = LCD_SendCmd(&cmdMsg);
        break;
    default:
        xReturn = pdFALSE;
        break;
    }

    va_end(argList);
    return xReturn;
}
