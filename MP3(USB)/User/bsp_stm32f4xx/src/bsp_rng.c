#include "bsp.h"

/*
*********************************************************************************************************
*	函 数 名: RNG_Config
*	功能说明: 配置生成随机数
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
void bsp_RNG_Config(void)
{  
 /* Enable RNG clock source */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

  /* RNG Peripheral enable */
  RNG_Cmd(ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: RNG_Config
*	功能说明: 配置生成随机数
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
uint32_t bsp_Generate_Random(void)
{
	uint32_t random32bit;
	/* Wait until one RNG number is ready */
	while(RNG_GetFlagStatus(RNG_FLAG_DRDY)== RESET)
  {
  }

   /* Get a 32bit Random number */       
   random32bit = RNG_GetRandomNumber();
	
	return random32bit;
}

