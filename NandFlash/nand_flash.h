/*
 * nand_flash.h
 *
 *  Created on: 06-02-2013
 *      Author: Grzybek
 */

#ifndef NAND_FLASH_H_
#define NAND_FLASH_H_

#include "stm32f10x.h"

/*
 * Defines
 */
typedef struct
{
    uint8_t Maker_ID;
    uint8_t Device_ID;
    uint8_t Third_ID;
    uint8_t Fourth_ID;
    uint8_t Fifth_ID;
}NAND_IDTypeDef;

typedef struct
{
  u16 Zone;
  u16 Block;
  u16 Page;
} NAND_ADDRESS;

/* Exported constants --------------------------------------------------------*/
/* NAND Area definition  for STM3210E-EVAL Board RevD */
#define CMD_AREA                   (u32)(1<<16)  /* A16 = CLE  high */
#define ADDR_AREA                  (u32)(1<<17)  /* A17 = ALE high */

#define DATA_AREA                  ((u32)0x00000000)
/* FSMC NAND memory address computation */
#define ADDR_1st_CYCLE(ADDR)       (u8)((ADDR)& 0xFF)               /* 1st addressing cycle */
#define ADDR_2nd_CYCLE(ADDR)       (u8)(((ADDR)& 0xFF00) >> 8)      /* 2nd addressing cycle */
#define ADDR_3rd_CYCLE(ADDR)       (u8)(((ADDR)& 0xFF0000) >> 16)   /* 3rd addressing cycle */
#define ADDR_4th_CYCLE(ADDR)       (u8)(((ADDR)& 0xFF000000) >> 24) /* 4th addressing cycle */

/* FSMC NAND memory command */
#define NAND_CMD_READ_1             ((u8)0x00)
#define NAND_CMD_READ_TRUE          ((u8)0x30)

#define NAND_CMD_RDCOPYBACK         ((u8)0x00)
#define NAND_CMD_RDCOPYBACK_TRUE    ((u8)0x35)

#define NAND_CMD_PAGEPROGRAM        ((u8)0x80)
#define NAND_CMD_PAGEPROGRAM_TRUE   ((u8)0x10)

#define NAND_CMD_COPYBACKPGM        ((u8)0x85)
#define NAND_CMD_COPYBACKPGM_TRUE   ((u8)0x10)

#define NAND_CMD_ERASE0             ((u8)0x60)
#define NAND_CMD_ERASE1             ((u8)0xD0)

#define NAND_CMD_READID             ((u8)0x90)
#define NAND_CMD_STATUS             ((u8)0x70)
#define NAND_CMD_RESET              ((u8)0xFF)

#define NAND_CMD_CACHEPGM           ((u8)0x80)
#define NAND_CMD_CACHEPGM_TRUE      ((u8)0x15)

#define NAND_CMD_RANDOMIN           ((u8)0x85)
#define NAND_CMD_RANDOMOUT          ((u8)0x05)
#define NAND_CMD_RANDOMOUT_TRUE     ((u8)0xE0)

#define NAND_CMD_CACHERD_START      ((u8)0x00)
#define NAND_CMD_CACHERD_START2     ((u8)0x31)
#define NAND_CMD_CACHERD_EXIT       ((u8)0x34)

/* NAND memory status */
#define NAND_VALID_ADDRESS         ((u32)0x00000100)
#define NAND_INVALID_ADDRESS       ((u32)0x00000200)
#define NAND_TIMEOUT_ERROR         ((u32)0x00000400)
#define NAND_BUSY                  ((u32)0x00000000)
#define NAND_ERROR                 ((u32)0x00000001)
#define NAND_READY                 ((u32)0x00000040)

/* FSMC NAND memory parameters */
#define NAND_PAGE_SIZE             ((u16)0x0800) /* 2K bytes per page w/o Spare Area */
#define NAND_BLOCK_SIZE            ((u16)0x0040) /* 64x2K bytes pages per block */
#define NAND_ZONE_SIZE             ((u16)0x0400) /* 1024 Block per zone */
#define NAND_SPARE_AREA_SIZE       ((u16)0x0040) /* last 64 bytes as spare area */
#define NAND_MAX_ZONE              ((u16)0x0001) /* 1 zones of 1024 block */


/* FSMC NAND memory address computation */
#define ADDR_1st_CYCLE(ADDR)       (u8)((ADDR)& 0xFF)               /* 1st addressing cycle */
#define ADDR_2nd_CYCLE(ADDR)       (u8)(((ADDR)& 0xFF00) >> 8)      /* 2nd addressing cycle */
#define ADDR_3rd_CYCLE(ADDR)       (u8)(((ADDR)& 0xFF0000) >> 16)   /* 3rd addressing cycle */
#define ADDR_4th_CYCLE(ADDR)       (u8)(((ADDR)& 0xFF000000) >> 24) /* 4th addressing cycle */


void FSMC_NAND_Init(void);
void FSMC_NAND_ReadID(NAND_IDTypeDef* NAND_ID);
u32 FSMC_NAND_EraseBlock(NAND_ADDRESS Address);
u32 FSMC_NAND_WriteSmallPage(u8 *pBuffer, NAND_ADDRESS Address, u32 NumPageToWrite);
u32 FSMC_NAND_ReadSmallPage(u8 *pBuffer, NAND_ADDRESS Address, u32 NumPageToRead);
u32 FSMC_NAND_AddressIncrement(NAND_ADDRESS* Address);
u32 FSMC_NAND_GetStatus(void);
u32 FSMC_NAND_ReadStatus(void);

#endif /* NAND_FLASH_H_ */
