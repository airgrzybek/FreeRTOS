/*
 * touch_driver.h
 *
 *  Created on: 10-02-2013
 *      Author: Grzybek
 */

#ifndef TOUCH_DRIVER_H_
#define TOUCH_DRIVER_H_

#include "stm32f10x.h"

/*
 * SPI configuration setup
 */
#define TOUCH_SPI           SPI2
#define SPI_PORT            GPIOB
#define SPI_MISO            GPIO_Pin_14
#define SPI_MOSI            GPIO_Pin_15
#define SPI_SCK             GPIO_Pin_13
#define SPI_NSS             GPIO_Pin_12

#define PENIRQ_PORT         GPIOG
#define PENIRQ_PIN          GPIO_Pin_7

#define S                   1 << 7 // start bit
#define SER_DFR             0 << 2// differential mode
#define MODE                0 << 3// 12 bit conversion

/*
 * Define power down mode enabled
 */
#define PD1                 0 << 1
#define PD0                 0 << 0

#define X_AXIS              0x90
#define Y_AXIS              0xD0

typedef enum
{
    X = 1,
    Y = 5,
}TouchAxis;

typedef enum
{
    TouchStatus_StatusOk,
    TouchStatus_StatusNok,
    TouchStatus_DeviceFailure
}TouchStatus;

void Touch_DriverInit(void);
void Touch_PinConfiguration(void);
TouchStatus Touch_Read(TouchAxis axis,volatile u16 * data);
uint8_t Touch_GetPenState(void);

#endif /* TOUCH_DRIVER_H_ */
