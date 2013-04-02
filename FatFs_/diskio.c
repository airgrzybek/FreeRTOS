/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include "sdcard.h"
#include "string.h"
/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */
SD_CardInfo SDCardInfo;
#define BlockSize            512 /* Block Size in Bytes [这里的block含义是“扇区”，SD卡的每个扇区为512字节]*/
#define BufferWordsSize      (BlockSize >> 2)	//128，含义参考MAIN中的解释，是128个32位的缓冲区，存放512个字节
//u32 buff2[512/4];


/*-----------------------------------------------------------------------*/
/* Inidialize a Drive [该函数是需要改写的]                                                   */
/* 注意：从函数的代码上来看，所谓的drv这个参数，是用一个数字来代表文件硬件的类型，从其switch语句看
   drv = ATA = 0：表示是IDE硬盘的一种接口
   drv = MMC = 1: 表示的是MMC卡的接口，也就是SD卡
   drv = USB = 2: 表示的是USB存储设备的接口
   从这里看来，我们应该要选择的是MMC接口,输入参数必须为1*/
DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{	
  SD_Error Status;
  Status = SD_Init();
  if (Status == SD_OK)
  {
    /*----------------- Read CSD/CID MSD registers ------------------*/
    Status = SD_GetCardInfo(&SDCardInfo);
  }

  if (Status == SD_OK)
  {
    /*----------------- Select Card --------------------------------*/
    Status = SD_SelectDeselect((u32) (SDCardInfo.RCA << 16));
  }

  if (Status == SD_OK)
  {
    Status = SD_EnableWideBusOperation(SDIO_BusWide_4b);
  }

  /* Set Device Transfer Mode to DMA [设置传输模式为DMA方式]*/
  if (Status == SD_OK)
  {  
    Status = SD_SetDeviceMode(SD_DMA_MODE);
  }
  if(Status == SD_OK)return 0;
  else return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status  [该函数需要改写]                                                  */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s) [该函数需要改写]                                       */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
  //memset(buff2, 0, sizeof(buff2));
	if(count==1)
        {
          SD_ReadBlock(sector << 9 ,(u32 *)(&buff[0]),BlockSize);
          //memcpy(buff,buff2,SECTOR_SIZE);
	}
	else
        {
          SD_ReadMultiBlocks(sector << 9 ,(u32 *)(&buff[0]),BlockSize,count);
          //memcpy(buff,buff2,SECTOR_SIZE * count);
	}
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s) [该函数需要改写]                                                      */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
  //memset(buff2, 0, sizeof(buff2));
	if(count==1)
        {
          //memcpy(buff2,buff,SECTOR_SIZE);
          SD_WriteBlock(sector << 9 ,(u32 *)(&buff[0]),BlockSize);
	}
	else
        {
          //memcpy(buff2,buff,SECTOR_SIZE * count);
          SD_WriteMultiBlocks(sector << 9 ,(u32 *)(&buff[0]),BlockSize,count);
	}
        
  return RES_OK;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions [该函数需要改写]                                              */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	return RES_OK;
}

