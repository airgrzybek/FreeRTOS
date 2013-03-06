/*
 * FSMC_config.h
 *
 *  Created on: 07-02-2013
 *      Author: Grzybek
 */

#ifndef FSMC_CONFIG_H_
#define FSMC_CONFIG_H_

#include "stm32f10x.h"




void FSMC_Init();
void FSMC_PinConfiguration();
void FSMC_LcdInit();
void FSMC_NorFlashInit();
void FSMC_NandFlashInit();
void FSMC_SramInit();

#endif /* FSMC_CONFIG_H_ */
