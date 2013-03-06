/*
 * FreeRTOS_stm32f10x_usart.c
 *
 *  Created on: 30-01-2013
 *      Author: Grzybek
 */


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* IO library includes. */
#include "FreeRTOS_IO.h"
#include "IOUtils_Common.h"
#include "FreeRTOS_uart.h"

#include "stm32f10x.h"

#include "stm32DragonBoard.h"

/* Stores the transfer control structures that are currently in use by the
supported UART ports. */
static Transfer_Control_t *pxTxTransferControlStructs[ BOARD_NUM_USARTS ] = { NULL };
static Transfer_Control_t *pxRxTransferControlStructs[ BOARD_NUM_USARTS ] = { NULL };

/* Stores the IRQ numbers of the supported UART ports. */
static const IRQn_Type xIRQ[] = { USART1_IRQn, USART2_IRQn, USART3_IRQn };


portBASE_TYPE FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl )
{
    portBASE_TYPE xReturn = pdPASS;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_InitClockStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_TypeDef * USARTx = (USART_TypeDef *)diGET_PERIPHERAL_BASE_ADDRESS(pxPeripheralControl);

    const uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );

    if(cPeripheralNumber <= BOARD_NUM_USARTS)
    {
        pxPeripheralControl->read = FreeRTOS_UART_read;
        pxPeripheralControl->write = FreeRTOS_UART_write;
        pxPeripheralControl->ioctl = FreeRTOS_UART_ioctl;

        portENTER_CRITICAL();

        switch(cPeripheralNumber)
        {
        case 1:
            /* Enable USART1 clock */
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,
                    ENABLE);

            /* Configure USART1 Rx (PA10) as input floating */
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

            /* Configure USART1 Tx (PA9) as alternate function push-pull */
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;

        case 2:
            /* Enable USART2 clock */
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);


            /* Configure USART2 Rx (PA10) as input floating */
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
            GPIO_Init(GPIOA, &GPIO_InitStructure);

            /* Configure USART2 Tx (PA9) as alternate function push-pull */
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            break;

        default:
            return pdFALSE;
        }



        USART_InitStructure.USART_BaudRate = BOARD_DEF_BAUDRATE;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl =
                USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_InitClockStructure.USART_Clock = USART_Clock_Disable;
        USART_InitClockStructure.USART_CPOL = USART_CPOL_Low;
        USART_InitClockStructure.USART_CPHA = USART_CPHA_2Edge;
        USART_InitClockStructure.USART_LastBit = USART_LastBit_Disable;

        USART_Init(USARTx, &USART_InitStructure);
        USART_ClockInit(USARTx, &USART_InitClockStructure);

        USART_Cmd(USARTx, ENABLE);

        portEXIT_CRITICAL();
        xReturn = pdPASS;
    }
    else
    {
        xReturn = pdFAIL;
    }

    return xReturn;
}

size_t USART_SendBuffer(USART_TypeDef * USARTx, uint8_t * data, size_t bytes)
{
    size_t i = 0;

    for(i=0;i<bytes;i++)
    {
        while(USART_GetFlagStatus( USARTx, USART_FLAG_TXE ) == RESET);
        USART_SendData(USARTx,data[i]);
    }
    return --i;
}

size_t USART_ReceiveBuffer(USART_TypeDef * USARTx, uint8_t * data, size_t bytes)
{
    size_t i = 0;

    for(i=0;i<bytes;i++)
    {
        while(USART_GetFlagStatus( USARTx, USART_FLAG_RXNE ) == RESET);
        data[i] = USART_ReceiveData(USARTx);
    }
    return --i;
}

size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
    size_t xReturn = 0U;
    int8_t cPeripheralNumber;
    Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
    USART_TypeDef * USARTx = (USART_TypeDef *)diGET_PERIPHERAL_BASE_ADDRESS(pxPeripheralControl);

    if( diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
        {
            #if ioconfigUSE_UART_POLLED_TX == 1
            {
                portENTER_CRITICAL();
                xReturn = USART_SendBuffer(USARTx,(uint8_t*)pvBuffer,xBytes);
                portEXIT_CRITICAL();
            }
            #endif /* ioconfigUSE_UART_POLLED_TX */
        }
        else
        {
            /* Remember which transfer control structure is being used.
            The Tx interrupt will use this to continue to write data to the
            Tx FIFO/UART until the length member of the structure reaches
            zero. */
            cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
            pxTxTransferControlStructs[ cPeripheralNumber  ] = diGET_TX_TRANSFER_STRUCT( pxPeripheralControl );

            switch( diGET_TX_TRANSFER_TYPE( pxPeripheralControl ) )
            {
                case ioctlUSE_ZERO_COPY_TX :

                    #if ioconfigUSE_UART_ZERO_COPY_TX == 1
                    {
                        /* The implementation of the zero copy write uses a semaphore
                        to indicate whether a write is complete (and so the buffer
                        being written free again) or not.  The semantics of using a
                        zero copy write dictate that a zero copy write can only be
                        attempted by a task, once the semaphore has been successfully
                        obtained by that task.  This ensure that only one task can
                        perform a zero copy write at any one time.  Ensure the semaphore
                        is not currently available, if this function has been called
                        without it being obtained first then it is an error. */
                        configASSERT( xIOUtilsGetZeroCopyWriteMutex( pxPeripheralControl, ioctlOBTAIN_WRITE_MUTEX, 0U ) == 0 );
                        xReturn = xBytes;
                        ioutilsINITIATE_ZERO_COPY_TX
                            (
                                pxPeripheralControl,
                                USART_ITConfig( USARTx, USART_IT_TXE, DISABLE ),  /* Disable peripheral function. */
                                USART_ITConfig( USARTx, USART_IT_TXE, ENABLE ),   /* Enable peripheral function. */
                                prvFillFifoFromBuffer( pxUART, ( uint8_t ** ) &( pvBuffer ), xBytes ), /* Write to peripheral function. */
                                pvBuffer,                       /* Data source. */
                                xReturn                         /* Number of bytes to be written. This will get set to zero if the write mutex is not held. */
                            );
                    }
                    #endif /* ioconfigUSE_UART_ZERO_COPY_TX */
                    break;


                case ioctlUSE_CHARACTER_QUEUE_TX :

                    #if ioconfigUSE_UART_TX_CHAR_QUEUE == 1
                    {
                        //USART_ITConfig( USART1, USART_IT_TXE, ENABLE );
                        /* The queue allows multiple tasks to attempt to write
                        bytes, but ensures only the highest priority of these tasks
                        will actually succeed.  If two tasks of equal priority
                        attempt to write simultaneously, then the application must
                        ensure mutual exclusion, as time slicing could result in
                        the strings being sent to the queue being interleaved. */
                        ioutilsBLOCKING_SEND_CHARS_TO_TX_QUEUE
                            (
                                pxPeripheralControl,
                                (USART_GetFlagStatus( USARTx, USART_FLAG_TXE ) == SET),  /* Peripheral busy condition. */
                                USARTx->DR = ucChar,               /* Peripheral write function. */
                                ( ( uint8_t * ) pvBuffer ),         /* Data source. */
                                xBytes,                             /* Number of bytes to be written. */
                                xReturn );

                    }
                    #endif /* ioconfigUSE_UART_TX_CHAR_QUEUE */
                    break;


                default :

                    /* Other methods can be implemented here.  For now set the
                    stored transfer structure back to NULL as nothing is being
                    sent. */
                    configASSERT( xReturn );
                    pxTxTransferControlStructs[ cPeripheralNumber ] = NULL;

                    /* Prevent compiler warnings when the configuration is set such
                    that the following parameters are not used. */
                    ( void ) pvBuffer;
                    ( void ) xBytes;
                    ( void ) USARTx;
                    break;
            }
        }

        return xReturn;
}

size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
    Peripheral_Control_t * const pxPeripheralControl =
            (Peripheral_Control_t * const ) pxPeripheral;
    size_t xReturn = 0U;
    size_t rChar = 0;
    USART_TypeDef * USARTx = (USART_TypeDef *)diGET_PERIPHERAL_BASE_ADDRESS(pxPeripheralControl);

    if( diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
    {
        #if ioconfigUSE_UART_POLLED_RX == 1
        {
            xReturn = USART_ReceiveBuffer(USARTx,(uint8_t*)pvBuffer,xBytes);
        }
        #endif /* ioconfigUSE_UART_POLLED_RX */
    }
    else
    {
        /* Sanity check the array index. */
        configASSERT( diGET_PERIPHERAL_NUMBER( pxPeripheralControl ) < ( int8_t ) ( sizeof( xIRQ ) / sizeof( IRQn_Type ) ) );

        switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
        {
            case ioctlUSE_CIRCULAR_BUFFER_RX :

                #if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
                {
                    /* There is nothing to prevent multiple tasks attempting to
                    read the circular buffer at any one time.  The implementation
                    of the circular buffer uses a semaphore to indicate when new
                    data is available, and the semaphore will ensure that only the
                    highest priority task that is attempting a read will actually
                    receive bytes. */
                    ioutilsRECEIVE_CHARS_FROM_CIRCULAR_BUFFER
                        (
                            pxPeripheralControl,
                            USART_ITConfig( USARTx, USART_IT_RXNE, DISABLE ), /* Disable peripheral. */
                            USART_ITConfig( USARTx, USART_IT_RXNE, ENABLE ),  /* Enable peripheral. */
                            ( ( uint8_t * ) pvBuffer ),                         /* Data destination. */
                            xBytes,                                             /* Bytes to read. */
                            xReturn                                             /* Number of bytes read. */
                        );
                }
                #endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
                break;


            case ioctlUSE_CHARACTER_QUEUE_RX :

                #if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
                {
                    /* The queue allows multiple tasks to attempt to read
                    bytes, but ensures only the highest priority of these
                    tasks will actually receive bytes.  If two tasks of equal
                    priority attempt to read simultaneously, then the
                    application must ensure mutual exclusion, as time slicing
                    could result in the string being received being partially
                    received by each task. */
                    xReturn = xIOUtilsReceiveCharsFromRxQueue( pxPeripheralControl,
                                ( uint8_t * ) pvBuffer, xBytes );
                }
                #endif /* ioconfigUSE_UART_RX_CHAR_QUEUE */
                break;


            default :

                /* Other methods can be implemented here. */
                configASSERT( xReturn );

                /* Prevent compiler warnings when the configuration is set such
                that the following parameters are not used. */
                ( void ) pvBuffer;
                ( void ) xBytes;
                ( void ) USARTx;
                break;
        }
    }

    return xReturn;
}

portBASE_TYPE FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
    Peripheral_Control_t * const pxPeripheralControl =
            (Peripheral_Control_t * const ) pxPeripheral;
    uint32_t ulValue = (uint32_t) pvValue;
    const int8_t cPeripheralNumber =
            diGET_PERIPHERAL_NUMBER( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
    USART_TypeDef * USARTx = (USART_TypeDef *)diGET_PERIPHERAL_BASE_ADDRESS(pxPeripheralControl);
    portBASE_TYPE xReturn = pdPASS;
    NVIC_InitTypeDef NVIC_InitStructure;
    USART_ClockInitTypeDef USART_InitClockStructure;
    USART_InitTypeDef USART_InitStructure;

    /* Sanity check the array index. */
    configASSERT( cPeripheralNumber < ( int8_t ) ( sizeof( xIRQ ) / sizeof( IRQn_Type ) ) );

    taskENTER_CRITICAL();
    {
        switch (ulRequest)
        {
        case ioctlUSE_INTERRUPTS:

            if (ulValue == pdFALSE)
            {
                NVIC_DisableIRQ(xIRQ[cPeripheralNumber]);
            }
            else
            {
                /* Enable the Rx and Tx interrupt. */
                USART_ITConfig( USARTx, USART_IT_RXNE, ENABLE );
                //USART_ITConfig( USART1, USART_IT_TXE, ENABLE );

                NVIC_InitStructure.NVIC_IRQChannel = xIRQ[cPeripheralNumber-1];
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
                        configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);

                /* If the Rx is configured to use interrupts, remember the
                 transfer control structure that should be used.  A reference
                 to the Tx transfer control structure is taken when a write()
                 operation is actually performed. */
                pxRxTransferControlStructs[cPeripheralNumber] =
                        pxPeripheralControl->pxRxControl;
            }
            break;

        case ioctlSET_SPEED:

            /* Set up the default UART configuration. */
            USART_InitStructure.USART_BaudRate = ulValue;
            USART_InitStructure.USART_WordLength = USART_WordLength_8b;
            USART_InitStructure.USART_StopBits = USART_StopBits_1;
            USART_InitStructure.USART_Parity = USART_Parity_No ;
            USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
            USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
            USART_InitClockStructure.USART_Clock = USART_Clock_Disable;
            USART_InitClockStructure.USART_CPOL = USART_CPOL_Low;
            USART_InitClockStructure.USART_CPHA = USART_CPHA_2Edge;
            USART_InitClockStructure.USART_LastBit = USART_LastBit_Disable;
            USART_ClockInit( USARTx, &USART_InitClockStructure);
            break;

        case ioctlSET_INTERRUPT_PRIORITY:

            /* The ISR uses ISR safe FreeRTOS API functions, so the priority
             being set must be lower than (ie numerically larger than)
             configMAX_LIBRARY_INTERRUPT_PRIORITY. */
            configASSERT( ulValue >= configMAX_LIBRARY_INTERRUPT_PRIORITY );
            //NVIC_SetPriority(xIRQ[cPeripheralNumber], ulValue);

            NVIC_InitStructure.NVIC_IRQChannel = xIRQ[cPeripheralNumber-1];
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ulValue;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
            break;

        default:

            xReturn = pdFAIL;
            break;
        }
    }
    taskEXIT_CRITICAL();

    return xReturn;
}

//(USART_GetFlagStatus( USART1, USART_FLAG_RXNE ) == RESET)
void USART1_IRQHandler(void)
{
    uint32_t ulInterruptSource, ulReceived;
    const uint32_t ulRxInterrupts = USART_IT_RXNE;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    const unsigned portBASE_TYPE uxUARTNumber = 1UL;
    Transfer_Control_t *pxTransferStruct;

    /* Determine the interrupt source. */

    if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)
    {
        pxTransferStruct = pxRxTransferControlStructs[ uxUARTNumber ];
        if( pxTransferStruct != NULL )
        {
            switch( diGET_TRANSFER_TYPE_FROM_CONTROL_STRUCT( pxTransferStruct ) )
            {
                case ioctlUSE_CIRCULAR_BUFFER_RX :

#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
                {
                    ioutilsRX_CHARS_INTO_CIRCULAR_BUFFER_FROM_ISR(
                            pxTransferStruct, /* The structure that contains the reference to the circular buffer. */
                            (USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET), /* While loop condition. */
                            (USART1->DR), /* Register holding the received character. */
                            ulReceived,
                            xHigherPriorityTaskWoken
                    );
                }
#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
                break;

                case ioctlUSE_CHARACTER_QUEUE_RX :

#if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
                {
                    ioutilsRX_CHARS_INTO_QUEUE_FROM_ISR( pxTransferStruct,
                            (USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET),
                            (USART1->DR), ulReceived, xHigherPriorityTaskWoken );
                }
#endif /* ioconfigUSE_UART_RX_CHAR_QUEUE */
                break;

                default :

                /* This must be an error.  Force an assert. */
                configASSERT( xHigherPriorityTaskWoken );
                break;
            }
        }
    }

    if(USART_GetITStatus(USART1,USART_IT_TXE) == SET)
    {
        /* The transmit holding register is empty.  Is there any more data
         to send? */
        pxTransferStruct = pxTxTransferControlStructs[ uxUARTNumber ];
        if( pxTransferStruct != NULL )
        {
            switch( diGET_TRANSFER_TYPE_FROM_CONTROL_STRUCT( pxTransferStruct ) )
            {
                case ioctlUSE_ZERO_COPY_TX:

#if ioconfigUSE_UART_ZERO_COPY_TX == 1
                {
                    iouitlsTX_CHARS_FROM_ZERO_COPY_BUFFER_FROM_ISR( pxTransferStruct, ( ( LPC_UART3->FIFOLVL & uartTX_FIFO_LEVEL_MASK ) != uartTX_FIFO_LEVEL_MASK ), ( LPC_UART3->THR = ucChar ), xHigherPriorityTaskWoken );
                }
#endif /* ioconfigUSE_UART_ZERO_COPY_TX */
                break;

                case ioctlUSE_CHARACTER_QUEUE_TX:

#if ioconfigUSE_UART_TX_CHAR_QUEUE == 1
                {
                    ioutilsTX_CHARS_FROM_QUEUE_FROM_ISR( pxTransferStruct,
                            (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == SET),
                            ( USART1->DR = ucChar), xHigherPriorityTaskWoken );
                    USART_ITConfig( USART1, USART_IT_TXE, DISABLE );
                }
#endif /* ioconfigUSE_UART_TX_CHAR_QUEUE */
                break;

                default :

                /* This must be an error.  Force an assert. */
                configASSERT( xHigherPriorityTaskWoken );
                break;
            }
        }
    }

    /* The ulReceived parameter is not used by the UART ISR. */
    ( void ) ulReceived;

    /* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
     switch should be performed before the interrupt exists.  That ensures the
     unblocked (higher priority) task is returned to immediately. */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void USART2_IRQHandler(void)
{
    uint32_t ulInterruptSource, ulReceived;
    const uint32_t ulRxInterrupts = USART_IT_RXNE;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    const unsigned portBASE_TYPE uxUARTNumber = 1UL;
    Transfer_Control_t *pxTransferStruct;

    /* Determine the interrupt source. */

    if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
    {
        pxTransferStruct = pxRxTransferControlStructs[ uxUARTNumber ];
        if( pxTransferStruct != NULL )
        {
            switch( diGET_TRANSFER_TYPE_FROM_CONTROL_STRUCT( pxTransferStruct ) )
            {
                case ioctlUSE_CIRCULAR_BUFFER_RX :

#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
                {
                    ioutilsRX_CHARS_INTO_CIRCULAR_BUFFER_FROM_ISR(
                            pxTransferStruct, /* The structure that contains the reference to the circular buffer. */
                            (USART_GetFlagStatus(USART2,USART_FLAG_RXNE) == SET), /* While loop condition. */
                            (USART2->DR), /* Register holding the received character. */
                            ulReceived,
                            xHigherPriorityTaskWoken
                    );
                }
#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
                break;

                case ioctlUSE_CHARACTER_QUEUE_RX :

#if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
                {
                    ioutilsRX_CHARS_INTO_QUEUE_FROM_ISR( pxTransferStruct,
                            (USART_GetFlagStatus(USART2,USART_FLAG_RXNE) == SET),
                            (USART2->DR), ulReceived, xHigherPriorityTaskWoken );
                }
#endif /* ioconfigUSE_UART_RX_CHAR_QUEUE */
                break;

                default :

                /* This must be an error.  Force an assert. */
                configASSERT( xHigherPriorityTaskWoken );
                break;
            }
        }
    }

    if(USART_GetITStatus(USART2,USART_IT_TXE) == SET)
    {
        /* The transmit holding register is empty.  Is there any more data
         to send? */
        pxTransferStruct = pxTxTransferControlStructs[ uxUARTNumber ];
        if( pxTransferStruct != NULL )
        {
            switch( diGET_TRANSFER_TYPE_FROM_CONTROL_STRUCT( pxTransferStruct ) )
            {
                case ioctlUSE_ZERO_COPY_TX:

#if ioconfigUSE_UART_ZERO_COPY_TX == 1
                {
                    iouitlsTX_CHARS_FROM_ZERO_COPY_BUFFER_FROM_ISR( pxTransferStruct, ( ( LPC_UART3->FIFOLVL & uartTX_FIFO_LEVEL_MASK ) != uartTX_FIFO_LEVEL_MASK ), ( LPC_UART3->THR = ucChar ), xHigherPriorityTaskWoken );
                }
#endif /* ioconfigUSE_UART_ZERO_COPY_TX */
                break;

                case ioctlUSE_CHARACTER_QUEUE_TX:

#if ioconfigUSE_UART_TX_CHAR_QUEUE == 1
                {
                    ioutilsTX_CHARS_FROM_QUEUE_FROM_ISR( pxTransferStruct,
                            (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == SET),
                            ( USART2->DR = ucChar), xHigherPriorityTaskWoken );
                    USART_ITConfig( USART1, USART_IT_TXE, DISABLE );
                }
#endif /* ioconfigUSE_UART_TX_CHAR_QUEUE */
                break;

                default :

                /* This must be an error.  Force an assert. */
                configASSERT( xHigherPriorityTaskWoken );
                break;
            }
        }
    }

    /* The ulReceived parameter is not used by the UART ISR. */
    ( void ) ulReceived;

    /* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
     switch should be performed before the interrupt exists.  That ensures the
     unblocked (higher priority) task is returned to immediately. */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
