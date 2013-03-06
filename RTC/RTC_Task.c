/*
 * rtc.c
 *
 *  Created on: 03-03-2013
 *      Author: Grzybek
 */

#include <stdio.h>
#include <string.h>
#include "RTC_Task.h"
#include "LcdTask.h"

xTaskHandle xRTCTaskHandle;

portBASE_TYPE vStartRtcTask(unsigned portBASE_TYPE uxPriority)
{
    portBASE_TYPE result = pdTRUE;

    // Create a task
    if(pdPASS == xTaskCreate( RtcTask, ( signed portCHAR * ) "RTC", RTC_TASK_STACK_SIZE,
            NULL, uxPriority, &xRTCTaskHandle))
    {
        result = pdTRUE;
    }
    else
    {
        result = pdFALSE;
    }

    return result;
}

portTASK_FUNCTION(RtcTask,parameters)
{
    uint32_t THH = 0, TMM = 0, TSS = 0;
    char buff[20];

    memset(buff,0x00,20);

    RTC_Config();

    while(1)
    {
        vTaskSuspend(NULL);
        RTC_GetTime(&THH,&TMM,&TSS);
        sprintf(buff,"Time:%02d:%02d:%02d",THH,TMM,TSS);
        LCD_Cmd(DISPLAY_LINE_COLUMN,buff,5,0,strlen(buff));
    }

}

void RTC_Config()
{
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    BKP_DeInit();

    /* Enable LSE */
    RCC_LSEConfig(RCC_LSE_ON);
    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}

void RTC_GetTime(uint32_t * thh,uint32_t * tmm, uint32_t * tss)
{
    uint32_t time = RTC_GetCounter();

    /* Reset RTC Counter when Time is 23:59:59 */
    if (time == 0x0001517F)
    {
       RTC_SetCounter(0x0);
       /* Wait until last write operation on RTC registers has finished */
       RTC_WaitForLastTask();
    }

    /* Compute  hours */
    *thh = time / 3600;
    /* Compute minutes */
    *tmm = (time % 3600) / 60;
    /* Compute seconds */
    *tss = (time % 3600) % 60;
}

void RTC_SetTime(uint32_t thh,uint32_t tmm, uint32_t tss)
{
    RTC_SetCounter(3600*thh+60*tmm+tss);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}
