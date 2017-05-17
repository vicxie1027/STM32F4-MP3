/*
*********************************************************************************************************
*
*	模块名称 : VS1053B MP3播放器
*	文件名称 : demo_mp3_player.c
*	版    本 : V1.0
*	说    明 : 该例程通过FatFS文件系统读取NAND Flash根目录下的MP3文件，并控制VS1053B解码器进行音频播放
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "ff.h"			/* FatFS文件系统模块*/
#include "demo_mp3_player.h"
#include "usbh_bsp_msc.h"

/* 自动播放指定磁盘指定目录下的MP3文件 */
#define MP3_PATH	"/MP3"	/* MP3文件存放的缺省文件夹， 根目录下的Music */

#define SONG_LIST_MAX	300

/* 仅允许本文件内调用的函数声明 */
static uint8_t Mp3Pro(void);
static void MP3HardInit(void);
static void FillSongList(void);
static int atoi(char *str);  
static uint16_t Generate_Index(void);
	
MP3_T g_tMP3;
PLAY_LIST_T g_tPlayList[SONG_LIST_MAX];

/* 访问Fatfs用到的全局变量 */
FATFS   g_fs;
FIL 	g_Mp3File;

/*
*********************************************************************************************************
*	函 数 名: DemoMP3
*	功能说明: MP3播放器主程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
	uint8_t ucKeyCode;	/* 按键代码 */
	uint8_t ucNextSong;
void DemoMP3(void)
{
	MP3HardInit();		/* 配置VS1053B硬件和WM8978硬件 */
	
	USBH_Init(&USB_OTG_Core,USB_OTG_HS_CORE_ID,&USB_Host,&USBH_MSC_cb,&USR_cb);
	
	while(1)
	{
		USBH_Process(&USB_OTG_Core,&USB_Host);
	}
}

/*
*********************************************************************************************************
*	函 数 名: ush_usr_fs_init
*	功能说明: MP3播放器初始化函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ush_usr_fs_init()
{
	FillSongList();		/* 搜索磁盘根目录下的MP3文件，并填充到播放列表数组 */

	g_tMP3.ucPauseEn = 0;	/* 缺省开始播放 */

	if (g_tMP3.ListCount > 0)
	{
		ucNextSong = 1;			/* 定位下一首歌曲的标志 */
	}
	else
	{
		ucNextSong = 0;
	}
	//配置初始化模式
	g_tMP3.Mode = RANDOM;
	
	//根据模式确认索引
	if(g_tMP3.Mode == RANDOM)
	{
		g_tMP3.ListIndex = Generate_Index();
	}
	else
	{
		g_tMP3.ListIndex = 0;
	}
}


extern 	USART_T g_tUsart1; 
/*
*********************************************************************************************************
*	函 数 名: ush_usr_action
*	功能说明: MP3播放器运行函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void ush_usr_action()
{		
	char IndexBuf[256];

		bsp_Idle();		/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */
	
		//串口选择歌曲
		if(g_tUsart1.ucRxFinish == FINISH)
		{
			memset(IndexBuf,0,256);					
			memcpy(IndexBuf,g_tUsart1.pRxBuf,g_tUsart1.usRxCount);	
			g_tMP3.ListIndex = atoi(IndexBuf);					
			
			g_tUsart1.ucRxFinish = 0;
			g_tUsart1.usRxCount = 0;
			memset(IndexBuf,0,256);
			memset(g_tUsart1.pRxBuf,0,g_tUsart1.usRxSize);
			
			ucNextSong = 1; //启动歌曲切换
		}
			
		/* 打开歌曲文件 */
		if (ucNextSong == 1)
		{
			ucNextSong = 0;

			/* 关闭上一个文件*/
			f_close(&g_Mp3File);

			/* 打开MUSIC目录下的mp3 文件 */
			{
				char FileName[256];
				FRESULT result;

				sprintf(FileName, "%s/%s",MP3_PATH,g_tPlayList[g_tMP3.ListIndex].LenFileName);
				result = f_open(&g_Mp3File, FileName, FA_OPEN_EXISTING | FA_READ);
				if (result !=  FR_OK)
				{
					printf("打开MP3文件失败, %d-%s\r\n",g_tMP3.ListIndex, g_tPlayList[g_tMP3.ListIndex].LenFileName);
				}
				else
				{
					printf("正在播放: %d-%s\r\n",g_tMP3.ListIndex, g_tPlayList[g_tMP3.ListIndex].LenFileName);
				}
			}
			g_tMP3.uiProgress = 0;	/* 进度 */
		}

		if (g_tMP3.ucPauseEn == 0)
		{
			if (Mp3Pro() == 1)
			{
				//播放完成，根据模式确认下一首歌索引
				if(g_tMP3.Mode == SIGNAL)
				{
					g_tMP3.ListIndex = g_tMP3.ListIndex;
				}
				else if(g_tMP3.Mode == LIST)
				{
					g_tMP3.ListIndex += 1;
				}
				else 
				{
					g_tMP3.ListIndex = Generate_Index();
				}
				ucNextSong = 1;											//打开下一首歌
			}
		}
}

/*
*********************************************************************************************************
*	函 数 名: MP3HardInit
*	功能说明: 配置MP3播放相关的硬件
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void MP3HardInit(void)
{
	/* 配置VS1053硬件 */
	{
		/* 等待芯片内部操作完成 */
		if (vs1053_WaitTimeOut())
		{
			/* 如果没有插VS1053B模块，DREQ口线将返回低电平，这是一种异常情况 */
			printf("没有检测到MP3模块硬件\r\n");
			return;
		}

		vs1053_Init();
		vs1053_SoftReset();

		/* 打印MP3解码芯片型号 */
		{
			char *pModel;

			switch (vs1053_ReadChipID())
			{
				case VS1001:
					pModel = "VS1001";
					break;

				case VS1011:
					pModel = "VS1011";
					break;

				case VS1002:
					pModel = "VS1002";
					break;

				case VS1003:
					pModel = "VS1003";
					break;

				case VS1053:
					pModel = "VS1053";
					break;

				case VS1033:
					pModel = "VS1033";
					break;

				case VS1103:
					pModel = "VS1103";
					break;

				default:
					pModel = "unknow";
					break;
			}
			printf(" 解码芯片型号 : %s\r\n", pModel);	/* 显示芯片型号 */
		}

		g_tMP3.ucVolume = 200; 			/* 缺省音量,越大声音越小 */
		vs1053_SetVolume(g_tMP3.ucVolume);

		vs1053_SetBASS(0, 0, 0, 0);		/* 高频和低音不增强 */
	}

}

/*
*********************************************************************************************************
*	函 数 名: FillSongList
*	功能说明: 填充歌曲播放列表。搜索磁盘根目录下的歌曲，最多10个
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void FillSongList(void)
{
	/* 访问Fatfs用到的全局变量 */
	FRESULT result;
	FILINFO FileInf;
	DIR DirInf;
	char path[128];
	
	char *fn;
	
#if _USE_LFN 
	static char lfn[_MAX_LFN +1];
	FileInf.lfname = lfn;
	FileInf.lfsize = sizeof(lfn);
#endif

	/* 挂载文件系统 */
	result = f_mount(FS_SD, &g_fs);	/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("挂载文件系统失败 (%d)\r\n", result);
	}

	/* 打开歌曲目录 */
	sprintf(path, "%s", MP3_PATH);
	result = f_opendir(&DirInf, path); 	/* path可以带盘符，最后一个字符不能是/ */
	if (result != FR_OK)
	{
		printf("Open Root Directory Error (%d)\r\n", result);
	}
	
	g_tMP3.ListIndex = 0;
	g_tMP3.ListCount = 0;	/* 歌曲个数 */
	
	printf("MP3 目录中的MP3文件:\r\n");
	printf("/***************************************/\r\n");
	
	for(;;)
	{
		result = f_readdir(&DirInf,&FileInf); 		/* 读取目录项，索引会自动下移 */
			
		if (result != FR_OK || FileInf.fname[0] == 0)
		{
			break;
		}

		if (FileInf.fname[0] == '.')	/* 表示目录 */
		{
			continue;
		}
	
		if (FileInf.fattrib != AM_DIR)
		{
			uint8_t Len;

			Len = strlen(FileInf.fname);
			if (Len >= 5)
			{
				if (memcmp(&FileInf.fname[Len - 3], "MP3", 3) == 0)
				{
					/* 复制MP3文件名到播放列表 */

#if _USE_LFN
					fn = *FileInf.lfname ? FileInf.lfname : FileInf.fname;
					strcpy(g_tPlayList[g_tMP3.ListCount].LenFileName, fn);
#else
					fn = FileInf.fname;
					strcpy(g_tPlayList[g_tMP3.ListCount].FileName, fn);
#endif
					
					g_tPlayList[g_tMP3.ListCount].FileSize = FileInf.fsize;
					printf("%d-%s\r\n",g_tMP3.ListCount,fn);
					g_tMP3.ListCount++;		/* 歌曲个数 */
					
					
					/* 如果MP3文件已填满，则退出 */
					if (g_tMP3.ListCount > SONG_LIST_MAX)
					{
						break;
					}
				}
			}
		}
	}
	printf("/***************************************/\r\n");
	
	if (g_tMP3.ListCount == 0)
	{
		printf("没有在根目录下找到 MP3 文件\r\n");
	}
	else
	{
		g_tMP3.MaxIndex = g_tMP3.ListCount;
	}
}

/*
*********************************************************************************************************
*	函 数 名: Mp3Pro
*	功能说明: MP3文件播放，在主程序while循环中调用. 每次向VS105B发送32字节。
*	形    参: 无
*	返 回 值: 0 表示正常播放; 1 表示文件播放完毕,主程序据此切换到下一首歌曲
*********************************************************************************************************
*/
static uint8_t Mp3Pro(void)
{
	uint32_t bw,i;
	char buf[32];

	/* 如果VS1003空闲，则写入新的数据 */
	if (vs1053_ReqNewData())
	{
		f_read(&g_Mp3File, &buf, 32, &bw);
		if (bw <= 0)
		{
			return 1;
		}

		/* 计算进度 */
		g_tMP3.uiProgress += bw;

		vs1053_PreWriteData();	/* 写数据准备，设置好片选 */
		for (i = 0; i < bw; i++)
		{
			vs1053_WriteData(buf[i]);
		}
	}
	return 0;
}
/*
*********************************************************************************************************
*	函 数 名: atoi
*	功能说明: 把数值字符串转化为数字
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static int atoi(char *str)        
{
 int i,sum;
 i=0;
 sum=0;
 while(str[i])            
 {
  sum=sum*10+(str[i] - 0x30);
	 i++;
 }
 return(sum);
}
/*
*********************************************************************************************************
*	函 数 名: Generate_Index
*	功能说明: 生成一个随机索引值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint16_t Generate_Index(void)
{
	uint16_t index;
	uint32_t random;
	
	random = bsp_Generate_Random();
	index = random % g_tMP3.MaxIndex;
	
	return index;
}

