/*
 * nand_flash_test.c
 *
 *  Created on: 07-02-2013
 *      Author: Grzybek
 */


#include "nand_flash_test.h"


NOR_IDTypeDef NOR_ID;
NAND_IDTypeDef NAND_ID;
NAND_ADDRESS WriteReadAddr;
static uint8_t TxBuffer[BUFFER_SIZE];
static uint8_t RxBuffer[BUFFER_SIZE];



u8 NandFlashTest(void)
{

    FSMC_NAND_ReadID(&NAND_ID);
    if ((NAND_ID.Maker_ID == 0XEC) && (NAND_ID.Device_ID == 0XF1))
    {
       return NAND_Test();
    }
    else
        return Test_Nok;

}

/*******************************************************************************
* Function name : Fill_Buffer
* Description   : Fill the buffer
* Input         : - pBuffer: pointer on the Buffer to fill
*                 - BufferSize: size of the buffer to fill
*                 - Offset: first value to fill on the Buffer
* Output param  : None
*******************************************************************************/
void Fill_Buffer(u8 *pBuffer, u16 BufferLenght, u32 Offset)
{
  u16 IndexTmp = 0;

  /* Put in global buffer same values */
  for (IndexTmp = 0; IndexTmp < BufferLenght; IndexTmp++ )
  {
    pBuffer[IndexTmp] = IndexTmp + Offset;
  }
}



TestStatus NAND_Test(void)
{
    NAND_ADDRESS WriteReadAddr;
    vu32 PageNumber = 1, WriteReadStatus = 0, status= 0;
    u32 j;
    /* NAND memory address to write to */
    WriteReadAddr.Zone = 0x00;
    WriteReadAddr.Block = 0x00;
    WriteReadAddr.Page = 0x00;

    /* Erase the NAND first Block */
    FSMC_NAND_EraseBlock(WriteReadAddr);

    /* Write data to FSMC NAND memory */
    /* Fill the buffer to send */
    Fill_Buffer(TxBuffer, BUFFER_SIZE , 0x20);

    FSMC_NAND_WriteSmallPage(TxBuffer, WriteReadAddr, PageNumber);  /*Ð´ÂúÒ»Ò³*/

    /* Read back the written data */
    FSMC_NAND_ReadSmallPage (RxBuffer, WriteReadAddr, PageNumber);

    /* Verify the written data */
    for(j = 0; j < BUFFER_SIZE; j++)
    {
        //printf("\r\n The RxBuffer[%d] is 0x%x \r\n", j, RxBuffer[j]);
        if(RxBuffer[j] != TxBuffer[j])
        {
            return Test_DeviceFailure;
        }
    }
    return Test_DeviceSuccess;
}
