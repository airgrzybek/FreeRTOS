/*
 * delay_us.h
 *
 *  Created on: 12-02-2013
 *      Author: Grzybek
 */

#ifndef DELAY_US_H_
#define DELAY_US_H_


#include "stm32f10x.h"

void delay_us_init(void);
void delay_us(u16 us);

#endif /* DELAY_US_H_ */
