/*
 * stm32_sdio_low_level.h
 *
 *  Created on: 19-02-2013
 *      Author: Grzybek
 */

#ifndef STM32_SDIO_LOW_LEVEL_H_
#define STM32_SDIO_LOW_LEVEL_H_

#include "stm32f10x.h"

/**
  * @brief  SD FLASH SDIO Interface
  */

#define SD_DETECT_PIN                    GPIO_Pin_11                 /* PF.11 */
#define SD_DETECT_GPIO_PORT              GPIOF                       /* GPIOF */
#define SD_DETECT_GPIO_CLK               RCC_APB2Periph_GPIOF

#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40018080)
/**
  * @brief  SDIO Intialization Frequency (400KHz max)
  */
#define SDIO_INIT_CLK_DIV                ((uint8_t)0xB2)
/**
  * @brief  SDIO Data Transfer Frequency (25MHz max)
  */
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x00)


void SD_LowLevel_DeInit(void);
void SD_LowLevel_Init(void);
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);
uint32_t SD_DMAEndOfTransferStatus(void);

#endif /* STM32_SDIO_LOW_LEVEL_H_ */
