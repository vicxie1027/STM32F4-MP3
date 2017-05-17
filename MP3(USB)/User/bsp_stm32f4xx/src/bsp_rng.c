#include "bsp.h"

/*
*********************************************************************************************************
*	�� �� ��: RNG_Config
*	����˵��: �������������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
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
*	�� �� ��: RNG_Config
*	����˵��: �������������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
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

