/*
 * stm32DragonBoard.h
 *
 *  Created on: 30-01-2013
 *      Author: Grzybek
 */

#ifndef STM32DRAGONBOARD_H_
#define STM32DRAGONBOARD_H_

#include "stm32f10x.h"

#define boardAVAILABLE_DEVICES_LIST                                             \
{                                                                               \
    { ( const int8_t * const ) "/USART1/", eUART_TYPE, ( void * )USART1},   \
    { ( const int8_t * const ) "/USART2/", eUART_TYPE, ( void * )USART2},   \
}

/*******************************************************************************
 * Map the FreeRTOS+IO interface to the LPC17xx specific functions.
 ******************************************************************************/
portBASE_TYPE vFreeRTOS_stm32f10x_PopulateFunctionPointers( const Peripheral_Types_t ePeripheralType, Peripheral_Control_t * const pxPeripheralControl );
#define boardFreeRTOS_PopulateFunctionPointers vFreeRTOS_stm32f10x_PopulateFunctionPointers

/*
 * USART configurations
 */

#define BOARD_NUM_USARTS 3
#define BOARD_DEF_BAUDRATE 115200
#define BOARD_DEF_USART USART1

#endif /* STM32DRAGONBOARD_H_ */
