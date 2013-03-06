/*
 * TouchTask.c
 *
 *  Created on: 12-02-2013
 *      Author: Grzybek
 */


#include "TouchTask.h"
#include "LcdTask.h"

xTaskHandle xTouchTaskHandle;

portBASE_TYPE vStartTouchTask(unsigned portBASE_TYPE uxPriority)
{
    portBASE_TYPE result = pdTRUE;


    if(pdPASS == xTaskCreate( TouchTask, ( signed portCHAR * ) "TOUCH", TOUCH_TASK_STACK_SIZE,
            NULL, uxPriority, &xTouchTaskHandle))
    {
        result = pdTRUE;
    }
    else
    {
        result = pdFALSE;
    }

    return result;
}


portTASK_FUNCTION(TouchTask,parameters)
{
    u16 rawX=0,rawY=0;
    u16 X = 0, Y = 0;
    u16 sample =0;
    char text[20];

    (void)parameters;
    Touch_DriverInit();

    for (;;)
    {
        vTaskSuspend(xTouchTaskHandle); // PENIRQ wakes up task

        do
        {

            getTouch(&X, &Y);

            sprintf(text, "x=%d, y=%d", X, Y);
            LCD_Cmd(DISPLAY_LINE, text, 0);

            // release mcu for lcd task
            vTaskDelay(100);
        } while (Bit_RESET == Touch_GetPenState());

    }
}

TouchStatus getTouch(u16 * xAxis, u16 * yAxis)
{
    TouchStatus status = TouchStatus_StatusOk;
    u16 rawDataX = 0, rawDataY = 0, meanX = 0, meanY = 0;
    u16 sample =0;


    for (sample = 0; sample < SAMPLE_RATE; ++sample)
    {
        Touch_Read(X,&rawDataX);
        Touch_Read(Y,&rawDataY);
        meanX += rawDataX;
        meanY += rawDataY;
    }

    *xAxis = meanX/SAMPLE_RATE;
    *yAxis = meanY/SAMPLE_RATE;

    return status;
}
