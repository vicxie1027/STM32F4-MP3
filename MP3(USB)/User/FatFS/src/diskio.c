/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"			/* FatFs lower layer API */
#include "bsp_sdio_sd.h"	/* SD卡底层驱动 */
#include "usbh_bsp_msc.h"

#define SECTOR_SIZE		512

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
static volatile DSTATUS Stat = STA_NOINIT;	/* Disk status */

extern USB_OTG_CORE_HANDLE          USB_OTG_Core;
extern USBH_HOST                     USB_Host;

DSTATUS disk_initialize (
                         BYTE drv		/* Physical drive number (0) */
                           )
{

			if(HCD_IsDeviceConnected(&USB_OTG_Core))
			{  
				Stat &= ~STA_NOINIT;
			}
			
			return Stat;

}


/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
                     BYTE drv		/* Physical drive number (0) */
                       )
{

			if (drv) return STA_NOINIT;		/* Supports only single drive */
			return Stat;

}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
                   BYTE drv,			/* Physical drive number (0) */
                   BYTE *buff,			/* Pointer to the data buffer to store read data */
                   DWORD sector,		/* Start sector number (LBA) */
                   BYTE count			/* Sector count (1..255) */
                     )
{
  BYTE status = USBH_MSC_OK;
  
			if (drv || !count) return RES_PARERR;
			if (Stat & STA_NOINIT) return RES_NOTRDY;
			
			
			if(HCD_IsDeviceConnected(&USB_OTG_Core))
			{  
				
				do
				{
					status = USBH_MSC_Read10(&USB_OTG_Core, buff,sector,512 * count);
					USBH_MSC_HandleBOTXfer(&USB_OTG_Core ,&USB_Host);
					
					if(!HCD_IsDeviceConnected(&USB_OTG_Core))
					{ 
						return RES_ERROR;
					}      
				}
				while(status == USBH_MSC_BUSY );
			}
			
			if(status == USBH_MSC_OK)
				return RES_OK;
			return RES_ERROR;

}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write (
                    BYTE drv,			/* Physical drive number (0) */
                    const BYTE *buff,	/* Pointer to the data to be written */
                    DWORD sector,		/* Start sector number (LBA) */
                    BYTE count			/* Sector count (1..255) */
                      )
{
  BYTE status = USBH_MSC_OK;
	
			if (drv || !count) return RES_PARERR;
			if (Stat & STA_NOINIT) return RES_NOTRDY;
			if (Stat & STA_PROTECT) return RES_WRPRT;
			
			
			if(HCD_IsDeviceConnected(&USB_OTG_Core))
			{  
				do
				{
					status = USBH_MSC_Write10(&USB_OTG_Core,(BYTE*)buff,sector,512 * count);
					USBH_MSC_HandleBOTXfer(&USB_OTG_Core, &USB_Host);
					
					if(!HCD_IsDeviceConnected(&USB_OTG_Core))
					{ 
						return RES_ERROR;
					}
				}
				
				while(status == USBH_MSC_BUSY );
				
			}
			
			if(status == USBH_MSC_OK)
				return RES_OK;
			return RES_ERROR;
}
#endif /* _READONLY == 0 */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL != 0
DRESULT disk_ioctl (
                    BYTE drv,		/* Physical drive number (0) */
                    BYTE ctrl,		/* Control code */
                    void *buff		/* Buffer to send/receive control data */
                      )
{
  DRESULT res = RES_OK;
  
			if (drv) return RES_PARERR;
			
			res = RES_ERROR;
			
			if (Stat & STA_NOINIT) return RES_NOTRDY;
			
			switch (ctrl) {
			case CTRL_SYNC :		/* Make sure that no pending write process */
				
				res = RES_OK;
				break;
				
			case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
				
				*(DWORD*)buff = (DWORD) USBH_MSC_Param.MSCapacity;
				res = RES_OK;
				break;
				
			case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
				*(WORD*)buff = 512;
				res = RES_OK;
				break;
				
			case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
				
				*(DWORD*)buff = 512;
				
				break;
				
				
			default:
				res = RES_PARERR;
			}
			return res;

}
#endif /* _USE_IOCTL != 0 */

/*
*********************************************************************************************************
*	函 数 名: get_fattime
*	功能说明: 获得系统时间，用于改写文件的创建和修改时间。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
DWORD get_fattime (void)
{
	/* 如果有全局时钟，可按下面的格式进行时钟转换. 这个例子是2013-01-01 00:00:00 */

	return	  ((DWORD)(2013 - 1980) << 25)	/* Year = 2013 */
			| ((DWORD)1 << 21)				/* Month = 1 */
			| ((DWORD)1 << 16)				/* Day_m = 1*/
			| ((DWORD)0 << 11)				/* Hour = 0 */
			| ((DWORD)0 << 5)				/* Min = 0 */
			| ((DWORD)0 >> 1);				/* Sec = 0 */
}

