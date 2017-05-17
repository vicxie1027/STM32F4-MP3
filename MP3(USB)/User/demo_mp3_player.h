/*
*********************************************************************************************************
*
*	ģ������ : VS1053B MP3����������
*	�ļ����� : demo_mp3_player.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _DEMO_MP3_PLAYER_H
#define _DEMO_MP3_PLAYER_H


typedef enum
{
	SIGNAL = 0,
	LIST,
	RANDOM,
}MP3_MODE_E;

/* ����һ������MP3�������Ľṹ��
����ȫ�ֱ�������
*/
typedef struct
{
	uint8_t ucMuteOn;			/* 0 : ������ 1: ���� */
	uint8_t ucVolume;			/* ��ǰ���� */
	uint32_t uiProgress;		/* ��ǰ����(�Ѷ�ȡ���ֽ���) */
	uint8_t ucPauseEn;			/* ��ͣʹ�� */

	uint16_t ListCount;			/* �����б�ĸ������� */
	uint16_t ListIndex;			/* ��ǰ�������� */
	
	uint16_t MaxIndex;
	MP3_MODE_E Mode;
}MP3_T;

/* �����б� */
typedef struct
{
	char FileName[13];		/* 8+3�ṹ�ļ��� */
	uint32_t FileSize;		/* �ļ����� */
	char LenFileName[256];	/* ���ļ���, ����������ʾ */
}PLAY_LIST_T;

/* ���ⲿ���õĺ������� */
void DemoMP3(void);
void ush_usr_fs_init(void);
void ush_usr_action(void);

#endif


