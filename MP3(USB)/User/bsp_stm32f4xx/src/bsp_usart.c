/**
  ******************************************************************************
  * @file    bsp_usart.c 
  * @author  vic_xie
  * @version V1.5.0
  * @date    24-Feb-2017
  * @brief   
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

//#define RCC_RS485		(RCC_AHB1Periph_GPIOB)
//#define RS485_TXE_PIN		(GPIO_Pin_2)
//#define RS485_TXE_PORT	(GPIOB)

//#define RS485_TX_EN()		(RS485_TXE_PORT->BSRRL |= RS485_TXE_PIN)
//#define RS485_RX_EN()		(RS485_TXE_PORT->BSRRH |= RS485_TXE_PIN)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if USART1_EN == 1
	USART_T g_tUsart1;
	static uint8_t ucTxBuf1[UART1_TX_BUF_SIZE];
	static uint8_t ucRxBuf1[UART1_RX_BUF_SIZE];
#endif

#if USART3_EN == 1
	USART_T g_tUsart3;
	static uint8_t ucTxBuf3[UART3_TX_BUF_SIZE];
	static uint8_t ucRxBuf3[UART3_RX_BUF_SIZE];
#endif

/* Private functions ---------------------------------------------------------*/
static void UartVarInit(void);
static void InitHardUart(void);
static void ConfigUartNVIC(void);

static void bsp_InitTIM7(uint16_t ARR,uint16_t PSC);
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitUart
*	����˵��: ���ڳ�ʼ��
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
void bsp_InitUart(void)
{
	UartVarInit();		/* �����ȳ�ʼ��ȫ�ֱ���,������Ӳ�� */

	InitHardUart();		/* ���ô��ڵ�Ӳ������(�����ʵ�) */

	ConfigUartNVIC();	/* ���ô����ж� */
	
	bsp_InitTIM7(CYCLE_TIME,84);
}
/*
*********************************************************************************************************
*	�� �� ��: UartVarInit
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
static void UartVarInit(void)
{
#if USART1_EN == 1
	g_tUsart1.pRxBuf = ucRxBuf1;
	g_tUsart1.usRxSize = UART1_RX_BUF_SIZE;
	g_tUsart1.usRxCount = 0;
	g_tUsart1.ucRxFinish = 0;
	g_tUsart1.ucCnt = 0;
	
	g_tUsart1.pTxBuf = ucTxBuf1;
	g_tUsart1.usTxSize = UART1_TX_BUF_SIZE;
	g_tUsart1.usTxCount = 0;
	g_tUsart1.ucTxFinish = 1;
#endif
	
#if USART3_EN == 1
	g_tUsart3.pRxBuf = ucRxBuf3;
	g_tUsart3.usRxSize = UART3_RX_BUF_SIZE;
	g_tUsart3.usRxCount = 0;
	g_tUsart3.ucRxFinish = 0;
	g_tUsart3.ucCnt = 0;
		
	g_tUsart3.pTxBuf = ucTxBuf3;
	g_tUsart3.usTxSize = UART3_TX_BUF_SIZE;
	g_tUsart3.usTxCount = 0;
	g_tUsart3.ucTxFinish = 1;
#endif
}
/*
*********************************************************************************************************
*	�� �� ��: InitHardUart
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
static void InitHardUart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;

#if USART1_EN == 1		
	/*�� DMA��ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* �� GPIO ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* �� UART ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* �� PA9 ӳ��Ϊ USART1_TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	/* �� PA10 ӳ��Ϊ USART1_RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* ���� USART Tx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* �������Ϊ���� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* �ڲ���������ʹ�� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ���� USART Rx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ��2���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = UART1_BAUD;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */

	USART_Cmd(USART1, ENABLE);		/* ʹ�ܴ��� */

	USART_ClearFlag(USART1, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);															// Enable USART1 DMA TX request	 												UART1ʹ��DMA����						 
  DMA_Cmd(DMA2_Stream7, ENABLE);                                              // Enable DMA2 Channel7 ȫ��DMA2 7ͨ��
	
#endif

#if USART3_EN == 1
	/*�� DMA��ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* �� GPIO ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* �� UART ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* �� PB10 ӳ��Ϊ USART3_TX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	/* �� PB11 ӳ��Ϊ USART3_RX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* ���� USART Tx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* �������Ϊ���� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* �ڲ���������ʹ�� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ���� USART Rx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ��2���� ���ô���Ӳ������ */
	USART_InitStructure.USART_BaudRate = UART3_BAUD;	/* ������ */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */

	USART_Cmd(USART3, ENABLE);		/* ʹ�ܴ��� */

	USART_ClearFlag(USART3, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
	
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);															// Enable USART1 DMA TX request	 												UART1ʹ��DMA����						 
  DMA_Cmd(DMA1_Stream3, ENABLE);                                              // Enable DMA2 Channel7 ȫ��DMA2 7ͨ��
#endif
}
/*
*********************************************************************************************************
*	�� �� ��: ConfigUartNVIC
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
static void ConfigUartNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#if USART1_EN == 1
	/* ʹ�ܴ���1�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel =  DMA2_Stream7_IRQn;   				// ָ��IRQͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   			// ��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          			// �����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             			// IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);                             			// ����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
#endif
	
#if USART3_EN == 1
	/* ʹ�ܴ���1�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel =  DMA1_Stream3_IRQn;   				// ָ��IRQͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   			// ��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          			// �����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             			// IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);                             			// ����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
#endif
}

static void bsp_InitTIM7(uint16_t ARR,uint16_t PSC)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);           			        // TIM7 clock enable	TIM7ʹ��

		TIM_TimeBaseStructure.TIM_Period = ARR - 1; 							                  // ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	1ms�Ķ�ʱ	 arr���Զ���װֵ   //ARR��ֵ  TIM_Period=SYSCLK(72000 000)/(Prescaler(35)+1)/(ClockDivision)/1000ms
		TIM_TimeBaseStructure.TIM_Prescaler = PSC - 1 ; 						                // ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ   	psc��ʱ��Ԥ��Ƶ��    ��ʱʱ�����=(1S/(SYSCLK/(Prescaler+1)))*ARRֵ         (1S/(72000 000/(36+1)))*1000=0.001S=1ms
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 				            // ����ʱ�ӷָ�:TDTS = Tck_tim                                     ARR=��ʱʱ��/(1S/(SYSCLK/(Prescaler+1)))  0.001S/(1S/(72000 000/(36+1))) =0.001/0.000 001=1000
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                  // TIM���ϼ���ģʽ
		TIM_TimeBaseStructure.TIM_RepetitionCounter=TIM_PSCReloadMode_Update;    		// TIM Ԥ��Ƶֵ�ڸ����¼�װ��,ֻ��TIM1��TIM8������
		TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);							                // ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
		
		TIM_ARRPreloadConfig(TIM7, DISABLE);									                      // ��ֹARRԤװ�ػ�����
		TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);								                    // ʹ��ָ����TIM�ж�(TIMx����,TIM�ж�Դ,TIMx�жϵ���״̬)		 												  
		TIM_Cmd(TIM7, ENABLE);  												                            // ʹ��TIMx����	 TIM_TypeDef* TIMx

		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;    		         		          // ָ��IRQͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   			          // ��ռ���ȼ�0��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;          			          // �����ȼ�3��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             			          // IRQͨ����ʹ��
		NVIC_Init(&NVIC_InitStructure);                             			          // ����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}
/*
*********************************************************************************************************
*	�� �� ��: Usart_Send
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
void Usart_Send(USART_TypeDef* _USARTx ,const char *_ucaBuf)
{
#if USART1_EN == 1
	if(_USARTx == USART1)
	{
		while(g_tUsart1.ucTxFinish != 1) ;               														
		memset(g_tUsart1.pTxBuf,0,g_tUsart1.usTxSize);																
    g_tUsart1.usTxCount = strlen(_ucaBuf);                                       
		memcpy(g_tUsart1.pTxBuf,_ucaBuf,g_tUsart1.usTxCount);                      
		Usart1_SendDMA();		
	}
#endif
	
#if USART3_EN == 1
	if(_USARTx == USART3)
	{
		while(g_tUsart3.ucTxFinish != 1) ;               														
		memset(g_tUsart3.pTxBuf,0,g_tUsart3.usTxSize);																
    g_tUsart3.usTxCount = strlen(_ucaBuf);                                       
		memcpy(g_tUsart3.pTxBuf,_ucaBuf,g_tUsart3.usTxCount);                      
		Usart3_SendDMA();				
	}
#endif
}
/*
*********************************************************************************************************
*	�� �� ��: Usart1_SendDMA
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
#if USART1_EN == 1
void Usart1_SendDMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA2_Stream7);                                              			//  ��DMA2������4ch 4�Ĵ�������Ϊ��ʼֵ
	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;                             //	����DMAͨ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;            		// 	DMA����USART1����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)g_tUsart1.pTxBuf;              // 	����DMA���ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                     //  ����DMA�ķ���	
	DMA_InitStructure.DMA_BufferSize = g_tUsart1.usTxCount;                      //  DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        		//  �����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 		//  �ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 		//	����DMA�������ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        			// 	����DMA�ڴ����ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           		// 	��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                    			// 	DMAͨ��ӵ�и����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;											//	ָ�����FIFOģʽ��ֱ��ģʽ������ָ������ �� ��ʹ��FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;						//	ָ����FIFO��ֵˮƽ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //	ָ����Burstת�������ڴ洫��
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //	ָ����Burstת��������Χת��
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);                           			// 	����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��

	DMA_Cmd(DMA2_Stream7, ENABLE);	                                     
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	g_tUsart1.ucTxFinish = 0;
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: Usart3_SendDMA
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
#if USART3_EN == 1
void Usart3_SendDMA(void)
{
	
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA1_Stream3);                                              			//  ��DMA1������3ch 4�Ĵ�������Ϊ��ʼֵ
	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;                             //	����DMAͨ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;            		// 	DMA����USART1����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)g_tUsart3.pTxBuf;              // 	����DMA���ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                     //  ����DMA�ķ���	
	DMA_InitStructure.DMA_BufferSize = g_tUsart3.usTxCount;                      //  DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        		//  �����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 		//  �ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 		//	����DMA�������ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        			// 	����DMA�ڴ����ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           		// 	��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                    			// 	DMAͨ��ӵ�и����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;											//	ָ�����FIFOģʽ��ֱ��ģʽ������ָ������ �� ��ʹ��FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;						//	ָ����FIFO��ֵˮƽ
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //	ָ����Burstת�������ڴ洫��
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //	ָ����Burstת��������Χת��
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);                           			// 	����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��

	DMA_Cmd(DMA1_Stream3, ENABLE);	                                     
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
	g_tUsart3.ucTxFinish = 0;
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: DMA2_Stream7_IRQHandler
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)!= RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);                           //	����жϱ�־λ
		DMA_Cmd(DMA2_Stream7, DISABLE);	                                       			// 	�ر�DMA
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);	 															//	ʹ��USART��������жϣ����������������ֽڡ�					
	}	
}
/*
*********************************************************************************************************
*	�� �� ��: DMA1_Stream3_IRQHandler
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)!= RESET)
	{
		DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);                           //	����жϱ�־λ
		DMA_Cmd(DMA1_Stream3, DISABLE);	                                       			// 	�ر�DMA
		USART_ITConfig(USART3, USART_IT_TC, ENABLE);	 															//	ʹ��USART��������жϣ����������������ֽڡ�					
	}	
}

/*
*********************************************************************************************************
*	�� �� ��: USART1_IRQHandler
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
#if USART1_EN == 1
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
	{  				
		if(g_tUsart1.usRxCount < g_tUsart1.usRxSize)
		{
			g_tUsart1.pRxBuf[g_tUsart1.usRxCount] = USART_ReceiveData(USART1) ;									 	
			g_tUsart1.usRxCount++ ;	                                                          
			g_tUsart1.ucCnt = ((1000000 / (UART1_BAUD / (8+2))) / CYCLE_TIME) * 3 ;
		}
	}
	
	if(USART_GetITStatus(USART1,USART_IT_TC) != RESET)
	{
		USART_ITConfig(USART1, USART_IT_TC, DISABLE);	 														
		g_tUsart1.ucTxFinish = 1 ;
	}
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: USART3_IRQHandler
*	����˵��: 
*	��    �Σ���
*	�� �� ֵ: 
*********************************************************************************************************
*/
#if USART3_EN == 1
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)
	{  				
		if(g_tUsart3.usRxCount < g_tUsart3.usRxSize)
		{
			g_tUsart3.pRxBuf[g_tUsart3.usRxCount] = USART_ReceiveData(USART3) ;									 	
			g_tUsart3.usRxCount++ ;	                                                          
			g_tUsart3.ucCnt = ((1000000 / (UART3_BAUD / (8+2))) / CYCLE_TIME) * 3 ;
		}
	}
	
	if(USART_GetITStatus(USART3,USART_IT_TC) != RESET)
	{
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);	 														
		g_tUsart3.ucTxFinish = 1 ;
	}
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: TIM7_Handler
*	����˵��: 
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);

#if USART1_EN == 1
		if(g_tUsart1.ucCnt > 0)							   			 		                         
		{
			g_tUsart1.ucCnt-- ;							   			   		                       
			if(g_tUsart1.ucCnt == 0)						   		   			                      
			{
				g_tUsart1.ucRxFinish = 1;                               	          
			}
		}		
#endif
		
#if USART3_EN == 1
		if(g_tUsart3.ucCnt > 0)							   			 		                         
		{
			g_tUsart3.ucCnt-- ;							   			   		                       
			if(g_tUsart3.ucCnt == 0)						   		   			                      
			{
				g_tUsart3.ucRxFinish = 1;                               	          
			}
		}		
#endif
	}
}

/*
*********************************************************************************************************
*	�� �� ��: fputc
*	����˵��: �ض���putc��������������ʹ��printf�����Ӵ���1��ӡ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	/* �ȴ����ͽ��� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
	return ch;
}

/*
*********************************************************************************************************
*	�� �� ��: fgetc
*	����˵��: �ض���getc��������������ʹ��getchar�����Ӵ���1��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
}



