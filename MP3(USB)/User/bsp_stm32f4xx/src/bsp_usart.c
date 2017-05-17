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
*	函 数 名: bsp_InitUart
*	功能说明: 串口初始化
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
void bsp_InitUart(void)
{
	UartVarInit();		/* 必须先初始化全局变量,再配置硬件 */

	InitHardUart();		/* 配置串口的硬件参数(波特率等) */

	ConfigUartNVIC();	/* 配置串口中断 */
	
	bsp_InitTIM7(CYCLE_TIME,84);
}
/*
*********************************************************************************************************
*	函 数 名: UartVarInit
*	功能说明: 
*	形    参：无
*	返 回 值: 错误代码(无需处理)
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
*	函 数 名: InitHardUart
*	功能说明: 
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
static void InitHardUart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;

#if USART1_EN == 1		
	/*打开 DMA　时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	/* 打开 GPIO 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* 打开 UART 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* 将 PA9 映射为 USART1_TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	/* 将 PA10 映射为 USART1_RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 第2步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART1_BAUD;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */

	USART_Cmd(USART1, ENABLE);		/* 使能串口 */

	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);															// Enable USART1 DMA TX request	 												UART1使用DMA请求						 
  DMA_Cmd(DMA2_Stream7, ENABLE);                                              // Enable DMA2 Channel7 全能DMA2 7通道
	
#endif

#if USART3_EN == 1
	/*打开 DMA　时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* 打开 GPIO 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* 打开 UART 时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* 将 PB10 映射为 USART3_TX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	/* 将 PB11 映射为 USART3_RX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 第2步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART3_BAUD;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */

	USART_Cmd(USART3, ENABLE);		/* 使能串口 */

	USART_ClearFlag(USART3, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
	
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);															// Enable USART1 DMA TX request	 												UART1使用DMA请求						 
  DMA_Cmd(DMA1_Stream3, ENABLE);                                              // Enable DMA2 Channel7 全能DMA2 7通道
#endif
}
/*
*********************************************************************************************************
*	函 数 名: ConfigUartNVIC
*	功能说明: 
*	形    参：无
*	返 回 值: 
*********************************************************************************************************
*/
static void ConfigUartNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#if USART1_EN == 1
	/* 使能串口1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel =  DMA2_Stream7_IRQn;   				// 指定IRQ通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   			// 先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          			// 从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             			// IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);                             			// 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
#endif
	
#if USART3_EN == 1
	/* 使能串口1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel =  DMA1_Stream3_IRQn;   				// 指定IRQ通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   			// 先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          			// 从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             			// IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);                             			// 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
#endif
}

static void bsp_InitTIM7(uint16_t ARR,uint16_t PSC)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);           			        // TIM7 clock enable	TIM7使能

		TIM_TimeBaseStructure.TIM_Period = ARR - 1; 							                  // 设置在下一个更新事件装入活动的自动重装载寄存器周期的值	1ms的定时	 arr：自动重装值   //ARR的值  TIM_Period=SYSCLK(72000 000)/(Prescaler(35)+1)/(ClockDivision)/1000ms
		TIM_TimeBaseStructure.TIM_Prescaler = PSC - 1 ; 						                // 设置用来作为TIMx时钟频率除数的预分频值   	psc：时钟预分频数    定时时间计算=(1S/(SYSCLK/(Prescaler+1)))*ARR值         (1S/(72000 000/(36+1)))*1000=0.001S=1ms
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 				            // 设置时钟分割:TDTS = Tck_tim                                     ARR=定时时间/(1S/(SYSCLK/(Prescaler+1)))  0.001S/(1S/(72000 000/(36+1))) =0.001/0.000 001=1000
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;                  // TIM向上计数模式
		TIM_TimeBaseStructure.TIM_RepetitionCounter=TIM_PSCReloadMode_Update;    		// TIM 预分频值在更新事件装入,只在TIM1和TIM8中有用
		TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);							                // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
		
		TIM_ARRPreloadConfig(TIM7, DISABLE);									                      // 禁止ARR预装载缓冲器
		TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);								                    // 使能指定的TIM中断(TIMx外设,TIM中断源,TIMx中断的新状态)		 												  
		TIM_Cmd(TIM7, ENABLE);  												                            // 使能TIMx外设	 TIM_TypeDef* TIMx

		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;    		         		          // 指定IRQ通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   			          // 先占优先级0级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;          			          // 从优先级3级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             			          // IRQ通道被使能
		NVIC_Init(&NVIC_InitStructure);                             			          // 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}
/*
*********************************************************************************************************
*	函 数 名: Usart_Send
*	功能说明: 
*	形    参：无
*	返 回 值: 
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
*	函 数 名: Usart1_SendDMA
*	功能说明: 
*	形    参：无
*	返 回 值: 
*********************************************************************************************************
*/
#if USART1_EN == 1
void Usart1_SendDMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA2_Stream7);                                              			//  将DMA2数据流4ch 4寄存器重设为初始值
	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;                             //	设置DMA通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;            		// 	DMA外设USART1基地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)g_tUsart1.pTxBuf;              // 	定义DMA中内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                     //  定义DMA的方向	
	DMA_InitStructure.DMA_BufferSize = g_tUsart1.usTxCount;                      //  DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        		//  外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 		//  内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 		//	设置DMA外设数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        			// 	设置DMA内存数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           		// 	工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                    			// 	DMA通道拥有高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;											//	指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;						//	指定了FIFO阈值水平
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //	指定的Burst转移配置内存传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //	指定的Burst转移配置外围转移
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);                           			// 	根据DMA_InitStruct中指定的参数初始化DMA的通道

	DMA_Cmd(DMA2_Stream7, ENABLE);	                                     
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	g_tUsart1.ucTxFinish = 0;
}
#endif
/*
*********************************************************************************************************
*	函 数 名: Usart3_SendDMA
*	功能说明: 
*	形    参：无
*	返 回 值: 
*********************************************************************************************************
*/
#if USART3_EN == 1
void Usart3_SendDMA(void)
{
	
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA1_Stream3);                                              			//  将DMA1数据流3ch 4寄存器重设为初始值
	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;                             //	设置DMA通道
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;            		// 	DMA外设USART1基地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)g_tUsart3.pTxBuf;              // 	定义DMA中内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                     //  定义DMA的方向	
	DMA_InitStructure.DMA_BufferSize = g_tUsart3.usTxCount;                      //  DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        		//  外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 		//  内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 		//	设置DMA外设数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        			// 	设置DMA内存数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           		// 	工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                    			// 	DMA通道拥有高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;											//	指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;						//	指定了FIFO阈值水平
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //	指定的Burst转移配置内存传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //	指定的Burst转移配置外围转移
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);                           			// 	根据DMA_InitStruct中指定的参数初始化DMA的通道

	DMA_Cmd(DMA1_Stream3, ENABLE);	                                     
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
	g_tUsart3.ucTxFinish = 0;
}
#endif
/*
*********************************************************************************************************
*	函 数 名: DMA2_Stream7_IRQHandler
*	功能说明: 
*	形    参：无
*	返 回 值: 
*********************************************************************************************************
*/
void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)!= RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);                           //	清除中断标志位
		DMA_Cmd(DMA2_Stream7, DISABLE);	                                       			// 	关闭DMA
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);	 															//	使能USART发送完成中断，发送完成最后两个字节。					
	}	
}
/*
*********************************************************************************************************
*	函 数 名: DMA1_Stream3_IRQHandler
*	功能说明: 
*	形    参：无
*	返 回 值: 
*********************************************************************************************************
*/
void DMA1_Stream3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3)!= RESET)
	{
		DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);                           //	清除中断标志位
		DMA_Cmd(DMA1_Stream3, DISABLE);	                                       			// 	关闭DMA
		USART_ITConfig(USART3, USART_IT_TC, ENABLE);	 															//	使能USART发送完成中断，发送完成最后两个字节。					
	}	
}

/*
*********************************************************************************************************
*	函 数 名: USART1_IRQHandler
*	功能说明: 
*	形    参：无
*	返 回 值: 
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
*	函 数 名: USART3_IRQHandler
*	功能说明: 
*	形    参：无
*	返 回 值: 
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
*	函 数 名: TIM7_Handler
*	功能说明: 
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: fputc
*	功能说明: 重定义putc函数，这样可以使用printf函数从串口1打印输出
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	/* 等待发送结束 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
	return ch;
}

/*
*********************************************************************************************************
*	函 数 名: fgetc
*	功能说明: 重定义getc函数，这样可以使用getchar函数从串口1输入数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int fgetc(FILE *f)
{
	/* 等待串口1输入数据 */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(USART1);
}



