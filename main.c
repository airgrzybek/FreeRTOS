/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_IO.h"
#include "FreeRTOS_CLI.h"

/* Library includes. */
#include "stm32f10x.h"
#include "stm32f10x_it.h"

/* Demo app includes. */
#include "flash.h"
//#include "comtest.h"
#include "LcdTask.h"
#include "console.h"
#include "CLI-commands.h"
#include "FSMC_config.h"
#include "nand_flash.h"
#include "nor_flash.h"
//#include "fsmc_sram.h"
#include "lcd.h"
#include "TouchTask.h"
#include "stm32_sdio_sd.h"
//#include "sdcard.h"

/*-----------------------------------------------------------*/

/* The time between cycles of the 'check' functionality (defined within the
 tick hook. */
#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
//#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainADC_TASK_PRIORITY               ( tskIDLE_PRIORITY + 1 )

/* The maximum number of message that can be waiting for display at any one
 time. */
#define mainLCD_QUEUE_SIZE					( 3 )

/* The check task uses the sprintf function so requires a little more stack. */
#define mainCHECK_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 350 )
#define mainLCD_TASK_STACK_SIZE				( configMINIMAL_STACK_SIZE + 350 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The period of the system clock in nano seconds.  This is used to calculate
 the jitter time in nano seconds. */
#define mainNS_PER_CLOCK					( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

#define mainCOM_TEST_BAUD_RATE 115200
#define mainCOM_TEST_LED 3
#define cmdPARAMTER_NOT_USED        ( ( void * ) 0 )
/*-----------------------------------------------------------*/

/* Used in the run time stats calculations. */
static uint32_t ulClocksPer10thOfAMilliSecond = 0UL;

/**
 * Configure the hardware for the demo.
 */
static void prvSetupHardware(void);
static void InterruptsSetup(void);




int main(void)
{
    prvSetupHardware();
    InterruptsSetup();

    FSMC_Init();
    delay_us_init();


    vStartConsoleTask();
    vStartLEDFlashTasks(mainFLASH_TASK_PRIORITY);
    //vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );
    vStartLcdTask(tskIDLE_PRIORITY + 1);
    vStartTouchTask(tskIDLE_PRIORITY + 2);
    vStartAdcTask(mainADC_TASK_PRIORITY);
    vStartRtcTask(tskIDLE_PRIORITY + 3);

    //xTaskCreate( usartTest, ( signed char * ) "usartTest", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY, ( xTaskHandle * ) NULL );
    //xTaskCreate( nandFlashTest, ( signed char * ) "nandFlashTest", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY, ( xTaskHandle * ) NULL );

    vRegisterCLICommands();
    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
     task. */
    return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
    RCC_APB2PeriphClockCmd(
            RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
            | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
            | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG
            | RCC_APB2Periph_AFIO, ENABLE);

    /* SPI2 Periph clock enable */
    //RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );
    //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    /* Set the Vector Table base address at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Configure HCLK clock as SysTick clock source. */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

    vParTestInitialise();
}
/*-----------------------------------------------------------*/

static void InterruptsSetup(void)
{
    EXTI_InitTypeDef  EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;

    /* Configure PA.00 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    /* Connect PEN EXTI Line to Key Button GPIO Pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);

    /* Configure PEN EXTI Line to generate an interrupt on falling edge */
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Connect PEN EXTI Line to Key Button GPIO Pin
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource7 );

    // Configure PEN EXTI Line to generate an interrupt on falling edge
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // interrupt init
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Generate software interrupt: simulate a falling edge applied on PEN EXTI line */
    //EXTI_GenerateSWInterrupt(EXTI_Line8);
    //EXTI_GenerateSWInterrupt(EXTI_Line7);
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle *pxTask,
        signed portCHAR *pcTaskName)
{
    for (;;)
        ;
}

void vMainConfigureTimerForRunTimeStats(void)
{
    /* How many clocks are there per tenth of a millisecond? */
    ulClocksPer10thOfAMilliSecond = configCPU_CLOCK_HZ / 10000UL;
}
/*-----------------------------------------------------------*/

uint32_t ulMainGetRunTimeCounterValue(void)
{
    uint32_t ulSysTickCounts, ulTickCount, ulReturn;
    const uint32_t ulSysTickReloadValue = (configCPU_CLOCK_HZ
            / configTICK_RATE_HZ ) - 1UL;
    volatile uint32_t * const pulCurrentSysTickCount =
            ((volatile uint32_t *) 0xe000e018);
    volatile uint32_t * const pulInterruptCTRLState =
            ((volatile uint32_t *) 0xe000ed04);
    const uint32_t ulSysTickPendingBit = 0x04000000UL;

    /* NOTE: There are potentially race conditions here.  However, it is used
     anyway to keep the examples simple, and to avoid reliance on a separate
     timer peripheral. */

    /* The SysTick is a down counter.  How many clocks have passed since it was
     last reloaded? */
    ulSysTickCounts = ulSysTickReloadValue - *pulCurrentSysTickCount;

    /* How many times has it overflowed? */
    ulTickCount = xTaskGetTickCountFromISR();

    /* Is there a SysTick interrupt pending? */
    if ((*pulInterruptCTRLState & ulSysTickPendingBit) != 0UL)
    {
        /* There is a SysTick interrupt pending, so the SysTick has overflowed
         but the tick count not yet incremented. */
        ulTickCount++;

        /* Read the SysTick again, as the overflow might have occurred since
         it was read last. */
        ulSysTickCounts = ulSysTickReloadValue - *pulCurrentSysTickCount;
    }

    /* Convert the tick count into tenths of a millisecond.  THIS ASSUMES
     configTICK_RATE_HZ is 1000! */
    ulReturn = (ulTickCount * 10UL);

    /* Add on the number of tenths of a millisecond that have passed since the
     tick count last got updated. */
    ulReturn += (ulSysTickCounts / ulClocksPer10thOfAMilliSecond);

    return ulReturn;
}



#ifdef DEBUG

/* Keep the linker happy. */
void assert_failed( unsigned portCHAR* pcFile, unsigned portLONG ulLine )
{
    for(;; )
    {
    }
}

#endif
