/**
  ******************************************************************************
  * @file     bsp_usart.h 
  * @author  vic_xie
  * @version V1.5.0
  * @date    24-Feb-2017
  * @brief   
  ******************************************************************************
**/
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART_H
#define __BSP_USART_H

#define FINISH			1

#define USART1_EN		1

#define USART3_EN 	1

#define CYCLE_TIME		10

#if USART1_EN == 1
	#define UART1_BAUD			115200
	#define UART1_TX_BUF_SIZE	1*1024
	#define UART1_RX_BUF_SIZE	1*1024	
#endif

#if USART3_EN == 1
	#define UART3_BAUD			115200
	#define UART3_TX_BUF_SIZE	1*1024
	#define UART3_RX_BUF_SIZE	1*1024	
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	COM1 = 0,
	COM2 = 1,
	COM3 = 2,
	COM4 = 3,
	COM5 = 4
}COM_PORT_E;
	

typedef struct usart
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	
	uint16_t usTxSize;
	uint16_t usRxSize;
	
	uint16_t usTxCount;
	uint16_t usRxCount;
	
	uint8_t ucTxFinish;
	uint8_t ucRxFinish;
	uint32_t ucCnt;								//两帧数据的时间差
	
//	void (*SendBefor)(void);		//用于485通信
//	void (*SendOver)(void);
//	void (*ReciveNew)(uint8_t _byte);
}USART_T;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void bsp_InitUart(void);
void Usart_Send(USART_TypeDef* _USARTx ,const char *_ucaBuf);
void Usart1_SendDMA(void);

void Usart3_SendDMA(void);

#endif
