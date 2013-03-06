/*
 * nand_flash_test.h
 *
 *  Created on: 07-02-2013
 *      Author: Grzybek
 */

#ifndef NAND_FLASH_TEST_H_
#define NAND_FLASH_TEST_H_

#include "nand_flash.h"

#define BUFFER_SIZE         0x400


typedef enum
{
    Test_Ok,
    Test_Nok,
    Test_DeviceFailure,
    Test_DeviceSuccess
}TestStatus;

TestStatus NandFlashTest(void);
void Fill_Buffer(u8 *pBuffer, u16 BufferLenght, u32 Offset);
TestStatus NAND_Test(void);




#endif /* NAND_FLASH_TEST_H_ */
