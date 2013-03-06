/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "usbdisk.h"	/* Example: USB drive control */
//#include "atadrive.h"	/* Example: ATA drive control */
//#include "sdcard.h"		/* Example: MMC/SDC contorl */
#include "stm32_sdio_sd.h"
//#include "sdcard.h"

/* Definitions of physical drive number for each media */
#define ATA		                    0
#define MMC		                    1
#define USB		                    2

#define DATA_BLOCK_SIZE             512

/*-----------------------------------------------------------------------*/
/* Private function declarations                                         */
/*-----------------------------------------------------------------------*/
DRESULT SD_IOCTL(BYTE cmd, void * buff);

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat = STA_NODISK;
	SD_Error sdResult;

	switch (pdrv) {
	case ATA :
		//result = ATA_disk_initialize();

		// translate the reslut code here

		return stat;

	case MMC :
		//result = MMC_disk_initialize();
	    sdResult = SD_Init();

	    if (SD_OK == sdResult)
        {
            stat = STA_NOINIT;
        }
	    else
	    {
	        stat = STA_NODISK;
	    }
		// translate the reslut code here

		return stat;

	case USB :
		//result = USB_disk_initialize();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat = STA_NOINIT;
	int result;

	switch (pdrv) {
	case ATA :
		//result = ATA_disk_status();

		// translate the reslut code here

		return stat;

	case MMC :
		//result = MMC_disk_status();

		// translate the reslut code here

		return stat;

	case USB :
		//result = USB_disk_status();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..128) */
)
{
	DRESULT res;
	int result;
	SD_Error sdResult = SD_OK;

	switch (pdrv) {
	case ATA :
		// translate the arguments here

		//result = ATA_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;

	case MMC :
	    sdResult = SD_ReadMultiBlocks(buff,sector,DATA_BLOCK_SIZE,count);

	    if(SD_OK == sdResult)
	    {
	        res = RES_OK;
	    }
	    else
	    {
	        res = RES_ERROR;
	    }

		return res;

	case USB :
		// translate the arguments here

		//result = USB_disk_read(buff, sector, count);

		// translate the reslut code here

		return res;
	}
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..128) */
)
{
	DRESULT res;
	int result;
	SD_Error sdResult = SD_OK;

	switch (pdrv) {
	case ATA :
		// translate the arguments here

		//result = ATA_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case MMC :
	    sdResult = SD_WriteMultiBlocks(buff,sector,DATA_BLOCK_SIZE,count);

        if(SD_OK == sdResult)
        {
            res = RES_OK;
        }
        else
        {
            res = RES_ERROR;
        }

		return res;

	case USB :
		// translate the arguments here

		//result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
	}
	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	case ATA :
		// pre-process here

		//result = ATA_disk_ioctl(cmd, buff);

		// post-process here

		return res;

	case MMC :

		return SD_IOCTL(cmd,buff);

	case USB :
		// pre-process here

		//result = USB_disk_ioctl(cmd, buff);

		// post-process here

		return res;
	}
	return RES_PARERR;
}

DRESULT SD_IOCTL(BYTE cmd, void * buff)
{
    DRESULT res = RES_OK;
    SD_CardInfo SDInfo;
    DWORD * paramBuff;

    switch (cmd)
    {
    case CTRL_SYNC:
        if (SD_TRANSFER_OK == SD_GetTransferState())
        {
            res = RES_OK;
        }
        else
        {
            res = RES_ERROR;
        }
        break;

    case GET_SECTOR_SIZE:
        res = RES_PARERR;
        break;

    case GET_SECTOR_COUNT:
        if (SD_OK == SD_GetCardInfo(&SDInfo))
        {
            *(DWORD*) buff = SDInfo.CardCapacity;
            res = RES_OK;
        }
        else
        {
            res = RES_ERROR;
        }
        break;

    case GET_BLOCK_SIZE:
        if (SD_OK == SD_GetCardInfo(&SDInfo))
        {
            *(DWORD*) buff = SDInfo.CardBlockSize;
            res = RES_OK;
        }
        else
        {
            res = RES_ERROR;
        }
        break;

    case CTRL_ERASE_SECTOR:
        paramBuff = (DWORD*)buff;

        if(SD_OK == SD_Erase(paramBuff[0],paramBuff[1]))
        {
            res = RES_OK;
        }
        else
        {
            res = RES_ERROR;
        }

        break;

    default:
        res = RES_PARERR;
        break;
    }

    return res;
}

/*-----------------------------------------------------------------------*/
/* User defined function to give a current time to fatfs module          */
/* 31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */
/* 15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */
DWORD get_fattime (void)
{

    return 0;
}

#endif
