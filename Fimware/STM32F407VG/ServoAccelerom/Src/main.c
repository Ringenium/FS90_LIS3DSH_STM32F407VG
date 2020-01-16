
#include "main.h"






//**********USE VARIABLE************//

static char DataSPI;         //data from LIS
static int16_t X;           //axis X
static int Ans;





//*********USe DEFINE*************//

#define ErrX 187
#define Factor 0.06




int main(void)
{

  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


  SystemClock_Config();
  MX_GPIO_Init();
  PWMInit ();
  SpiInit();
  UsartInit();







      //*************First installation LIS3DSH+FS90******************//

  	  DatToLIS(0x8F00);                                 //WHO AM I
  	  if(DataSPI==0x3F)
  	  {

  		 TIM2->CR1 |=  TIM_CR1_CEN;                     //on PWM

  	  	 LL_mDelay(10);


  	  	DatToLIS(0x2067);                                //CTRL_REG 4  x,y,z-en ODR-100Hz
  	  	 LL_mDelay(10);
  	  	DatToLIS(0x23C8);                                //CTRL_REG 3
  	  	 LL_mDelay(10);

  	    GPIOD->BSRR|=GPIO_BSRR_BS15;

  	  }
  	    else
  	  	GPIOD->BSRR|=GPIO_BSRR_BS14;                     //error LIS3DSH

 
 

  while (1)
  {
	  DatToLIS(0xA800);   //xl
	  X=DataSPI;
	  DatToLIS(0xA900);   //xh
	  X|=DataSPI<<8;

	  Ans = Convmg(DataSPI, X, Factor, ErrX);
	  Ans+=1200;
	  uint16_t step = Ans/34;
	  TIM2->CCR1 = step+30;



  }

}
















//***********************************Interrupt****************************//

void SPI1_IRQHandler(void)
{
	if(SPI1->SR & SPI_SR_RXNE)
	{

	DataSPI=SPI1->DR;

	//while(!(USART2->SR & USART_SR_TXE));              //раскоментить для контроля принимаемых данных из SPI1
	//Usart2ChTx(DataSPI);

    }
}

/*void USART2_IRQHandler(void)
{
	if(USART2->SR & USART_SR_RXNE)
	{
		USART2->SR &= ~USART_SR_RXNE;
		Status = F407RCom();
		if(Status==0x01)
		{
			GPIOD->BSRR|=GPIO_BSRR_BS13;
			GPIOD->BSRR|=GPIO_BSRR_BR14;
		}

		if(Status==0x02)
		{
			GPIOD->BSRR|=GPIO_BSRR_BS14;
			GPIOD->BSRR|=GPIO_BSRR_BR13;
		}*/



  //  }
//}









void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


