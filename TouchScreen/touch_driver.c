/*
 * touch_driver.c
 *
 *  Created on: 10-02-2013
 *      Author: Grzybek
 */

#include "touch_driver.h"
#include "delay_us.h"

#define SET_CS()        GPIO_SetBits(GPIOB,SPI_NSS)
#define CLEAR_CS()      GPIO_ResetBits(GPIOB,SPI_NSS)


void Touch_DriverInit()
{
    SPI_InitTypeDef   SPI_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    Touch_PinConfiguration();

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NS = (SPI_NSS_Soft);
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(TOUCH_SPI, &SPI_InitStructure);

    /* Enable TOUCH_SPI */
    SPI_Cmd(TOUCH_SPI, ENABLE);
}

void Touch_PinConfiguration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // SCK and MOSI
    GPIO_InitStructure.GPIO_Pin = SPI_MOSI | SPI_SCK;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI_PORT,&GPIO_InitStructure);

    // MISO
    GPIO_InitStructure.GPIO_Pin = SPI_MISO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

    // NSS
    GPIO_InitStructure.GPIO_Pin = SPI_NSS;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_PORT, &GPIO_InitStructure);

    // PENIRQ
/*    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

     Connect PEN EXTI Line to Key Button GPIO Pin
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource7 );

     Configure PEN EXTI Line to generate an interrupt on falling edge
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // interrupt init
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


     Generate software interrupt: simulate a falling edge applied on PEN EXTI line
    EXTI_GenerateSWInterrupt(EXTI_Line7);*/

}

TouchStatus Touch_Read(TouchAxis axis,volatile u16 * data)
{
    TouchStatus status = TouchStatus_StatusOk;
    u8 WriteData = 0;
    u8 ReceiveData = 0;

    //WriteData = S | (axis << 6) | MODE | SER_DFR | PD1 | PD0;
    WriteData = (axis == X ? X_AXIS : Y_AXIS);


    CLEAR_CS();

    while (SPI_I2S_GetFlagStatus(TOUCH_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(TOUCH_SPI,WriteData);

    while (SPI_I2S_GetFlagStatus(TOUCH_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    ReceiveData = SPI_I2S_ReceiveData(TOUCH_SPI);


    delay_us(20);

    while (SPI_I2S_GetFlagStatus(TOUCH_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(TOUCH_SPI,0x00);

    while (SPI_I2S_GetFlagStatus(TOUCH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

    ReceiveData = SPI_I2S_ReceiveData(TOUCH_SPI);

    *data = ReceiveData << 8;

    delay_us(1);

    while (SPI_I2S_GetFlagStatus(TOUCH_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(TOUCH_SPI,0x00);

    while (SPI_I2S_GetFlagStatus(TOUCH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

    ReceiveData = SPI_I2S_ReceiveData(TOUCH_SPI);

    *data |= ReceiveData;

    *data >>= 4;
    *data &= 0x0fff; // 12-bit data

    SET_CS();

    return status;
}

uint8_t Touch_GetPenState(void)
{
    return GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_7);
}
