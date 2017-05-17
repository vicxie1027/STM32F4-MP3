/*
*********************************************************************************************************
*
*	模块名称 : VS1053B MP3播放器例程
*	文件名称 : demo_mp3_player.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
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

/* 定义一个用于MP3播放器的结构体
便于全局变量操作
*/
typedef struct
{
	uint8_t ucMuteOn;			/* 0 : 静音， 1: 放音 */
	uint8_t ucVolume;			/* 当前音量 */
	uint32_t uiProgress;		/* 当前进度(已读取的字节数) */
	uint8_t ucPauseEn;			/* 暂停使能 */

	uint16_t ListCount;			/* 播放列表的歌曲个数 */
	uint16_t ListIndex;			/* 当前歌曲索引 */
	
	uint16_t MaxIndex;
	MP3_MODE_E Mode;
}MP3_T;

/* 歌曲列表 */
typedef struct
{
	char FileName[13];		/* 8+3结构文件名 */
	uint32_t FileSize;		/* 文件长度 */
	char LenFileName[256];	/* 长文件名, 可以用于显示 */
}PLAY_LIST_T;

/* 供外部调用的函数声明 */
void DemoMP3(void);
void ush_usr_fs_init(void);
void ush_usr_action(void);

#endif


