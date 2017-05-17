/*
*********************************************************************************************************
*
*	ģ������ : VS1053B MP3������
*	�ļ����� : demo_mp3_player.c
*	��    �� : V1.0
*	˵    �� : ������ͨ��FatFS�ļ�ϵͳ��ȡNAND Flash��Ŀ¼�µ�MP3�ļ���������VS1053B������������Ƶ����
*
*	�޸ļ�¼ :
*		�汾��  ����        ����     ˵��
*		V1.0    2013-02-01 armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "ff.h"			/* FatFS�ļ�ϵͳģ��*/
#include "demo_mp3_player.h"
#include "usbh_bsp_msc.h"

/* �Զ�����ָ������ָ��Ŀ¼�µ�MP3�ļ� */
#define MP3_PATH	"/MP3"	/* MP3�ļ���ŵ�ȱʡ�ļ��У� ��Ŀ¼�µ�Music */

#define SONG_LIST_MAX	300

/* �������ļ��ڵ��õĺ������� */
static uint8_t Mp3Pro(void);
static void MP3HardInit(void);
static void FillSongList(void);
static int atoi(char *str);  
static uint16_t Generate_Index(void);
	
MP3_T g_tMP3;
PLAY_LIST_T g_tPlayList[SONG_LIST_MAX];

/* ����Fatfs�õ���ȫ�ֱ��� */
FATFS   g_fs;
FIL 	g_Mp3File;

/*
*********************************************************************************************************
*	�� �� ��: DemoMP3
*	����˵��: MP3������������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
	uint8_t ucKeyCode;	/* �������� */
	uint8_t ucNextSong;
void DemoMP3(void)
{
	MP3HardInit();		/* ����VS1053BӲ����WM8978Ӳ�� */
	
	USBH_Init(&USB_OTG_Core,USB_OTG_HS_CORE_ID,&USB_Host,&USBH_MSC_cb,&USR_cb);
	
	while(1)
	{
		USBH_Process(&USB_OTG_Core,&USB_Host);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: ush_usr_fs_init
*	����˵��: MP3��������ʼ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ush_usr_fs_init()
{
	FillSongList();		/* �������̸�Ŀ¼�µ�MP3�ļ�������䵽�����б����� */

	g_tMP3.ucPauseEn = 0;	/* ȱʡ��ʼ���� */

	if (g_tMP3.ListCount > 0)
	{
		ucNextSong = 1;			/* ��λ��һ�׸����ı�־ */
	}
	else
	{
		ucNextSong = 0;
	}
	//���ó�ʼ��ģʽ
	g_tMP3.Mode = RANDOM;
	
	//����ģʽȷ������
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
*	�� �� ��: ush_usr_action
*	����˵��: MP3���������к���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void ush_usr_action()
{		
	char IndexBuf[256];

		bsp_Idle();		/* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */
	
		//����ѡ�����
		if(g_tUsart1.ucRxFinish == FINISH)
		{
			memset(IndexBuf,0,256);					
			memcpy(IndexBuf,g_tUsart1.pRxBuf,g_tUsart1.usRxCount);	
			g_tMP3.ListIndex = atoi(IndexBuf);					
			
			g_tUsart1.ucRxFinish = 0;
			g_tUsart1.usRxCount = 0;
			memset(IndexBuf,0,256);
			memset(g_tUsart1.pRxBuf,0,g_tUsart1.usRxSize);
			
			ucNextSong = 1; //���������л�
		}
			
		/* �򿪸����ļ� */
		if (ucNextSong == 1)
		{
			ucNextSong = 0;

			/* �ر���һ���ļ�*/
			f_close(&g_Mp3File);

			/* ��MUSICĿ¼�µ�mp3 �ļ� */
			{
				char FileName[256];
				FRESULT result;

				sprintf(FileName, "%s/%s",MP3_PATH,g_tPlayList[g_tMP3.ListIndex].LenFileName);
				result = f_open(&g_Mp3File, FileName, FA_OPEN_EXISTING | FA_READ);
				if (result !=  FR_OK)
				{
					printf("��MP3�ļ�ʧ��, %d-%s\r\n",g_tMP3.ListIndex, g_tPlayList[g_tMP3.ListIndex].LenFileName);
				}
				else
				{
					printf("���ڲ���: %d-%s\r\n",g_tMP3.ListIndex, g_tPlayList[g_tMP3.ListIndex].LenFileName);
				}
			}
			g_tMP3.uiProgress = 0;	/* ���� */
		}

		if (g_tMP3.ucPauseEn == 0)
		{
			if (Mp3Pro() == 1)
			{
				//������ɣ�����ģʽȷ����һ�׸�����
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
				ucNextSong = 1;											//����һ�׸�
			}
		}
}

/*
*********************************************************************************************************
*	�� �� ��: MP3HardInit
*	����˵��: ����MP3������ص�Ӳ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void MP3HardInit(void)
{
	/* ����VS1053Ӳ�� */
	{
		/* �ȴ�оƬ�ڲ�������� */
		if (vs1053_WaitTimeOut())
		{
			/* ���û�в�VS1053Bģ�飬DREQ���߽����ص͵�ƽ������һ���쳣��� */
			printf("û�м�⵽MP3ģ��Ӳ��\r\n");
			return;
		}

		vs1053_Init();
		vs1053_SoftReset();

		/* ��ӡMP3����оƬ�ͺ� */
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
			printf(" ����оƬ�ͺ� : %s\r\n", pModel);	/* ��ʾоƬ�ͺ� */
		}

		g_tMP3.ucVolume = 200; 			/* ȱʡ����,Խ������ԽС */
		vs1053_SetVolume(g_tMP3.ucVolume);

		vs1053_SetBASS(0, 0, 0, 0);		/* ��Ƶ�͵�������ǿ */
	}

}

/*
*********************************************************************************************************
*	�� �� ��: FillSongList
*	����˵��: �����������б��������̸�Ŀ¼�µĸ��������10��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void FillSongList(void)
{
	/* ����Fatfs�õ���ȫ�ֱ��� */
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

	/* �����ļ�ϵͳ */
	result = f_mount(FS_SD, &g_fs);	/* Mount a logical drive */
	if (result != FR_OK)
	{
		printf("�����ļ�ϵͳʧ�� (%d)\r\n", result);
	}

	/* �򿪸���Ŀ¼ */
	sprintf(path, "%s", MP3_PATH);
	result = f_opendir(&DirInf, path); 	/* path���Դ��̷������һ���ַ�������/ */
	if (result != FR_OK)
	{
		printf("Open Root Directory Error (%d)\r\n", result);
	}
	
	g_tMP3.ListIndex = 0;
	g_tMP3.ListCount = 0;	/* �������� */
	
	printf("MP3 Ŀ¼�е�MP3�ļ�:\r\n");
	printf("/***************************************/\r\n");
	
	for(;;)
	{
		result = f_readdir(&DirInf,&FileInf); 		/* ��ȡĿ¼��������Զ����� */
			
		if (result != FR_OK || FileInf.fname[0] == 0)
		{
			break;
		}

		if (FileInf.fname[0] == '.')	/* ��ʾĿ¼ */
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
					/* ����MP3�ļ����������б� */

#if _USE_LFN
					fn = *FileInf.lfname ? FileInf.lfname : FileInf.fname;
					strcpy(g_tPlayList[g_tMP3.ListCount].LenFileName, fn);
#else
					fn = FileInf.fname;
					strcpy(g_tPlayList[g_tMP3.ListCount].FileName, fn);
#endif
					
					g_tPlayList[g_tMP3.ListCount].FileSize = FileInf.fsize;
					printf("%d-%s\r\n",g_tMP3.ListCount,fn);
					g_tMP3.ListCount++;		/* �������� */
					
					
					/* ���MP3�ļ������������˳� */
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
		printf("û���ڸ�Ŀ¼���ҵ� MP3 �ļ�\r\n");
	}
	else
	{
		g_tMP3.MaxIndex = g_tMP3.ListCount;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: Mp3Pro
*	����˵��: MP3�ļ����ţ���������whileѭ���е���. ÿ����VS105B����32�ֽڡ�
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ��������; 1 ��ʾ�ļ��������,������ݴ��л�����һ�׸���
*********************************************************************************************************
*/
static uint8_t Mp3Pro(void)
{
	uint32_t bw,i;
	char buf[32];

	/* ���VS1003���У���д���µ����� */
	if (vs1053_ReqNewData())
	{
		f_read(&g_Mp3File, &buf, 32, &bw);
		if (bw <= 0)
		{
			return 1;
		}

		/* ������� */
		g_tMP3.uiProgress += bw;

		vs1053_PreWriteData();	/* д����׼�������ú�Ƭѡ */
		for (i = 0; i < bw; i++)
		{
			vs1053_WriteData(buf[i]);
		}
	}
	return 0;
}
/*
*********************************************************************************************************
*	�� �� ��: atoi
*	����˵��: ����ֵ�ַ���ת��Ϊ����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: Generate_Index
*	����˵��: ����һ���������ֵ
*	��    ��: ��
*	�� �� ֵ: ��
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

