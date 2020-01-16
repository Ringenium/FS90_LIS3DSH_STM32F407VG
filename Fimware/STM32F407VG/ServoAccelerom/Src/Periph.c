//********************* INIT PERIPH ***************//



#include"Periph.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"




//***************** PERIPH *****************//

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
  {
 // Error_Handler();
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();


  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(168000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(168000000);
}


void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);


  RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
  GPIOD->MODER|=GPIO_MODER_MODER15_0|GPIO_MODER_MODER14_0|GPIO_MODER_MODER13_0|GPIO_MODER_MODER12_0;


}



void PWMInit (void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;

	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;

	GPIOA->AFR[0] |=  GPIO_AFRL_AFRL0_0;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL0_1;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL0_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL0_3;

	GPIOA->MODER |=   GPIO_MODER_MODER0_1;

	GPIOA->OSPEEDR|= GPIO_OSPEEDR_OSPEED0_1;

	TIM2->PSC = 2000-1;

	TIM2->ARR = 850;

	TIM2->CCR1 = 58;                               //first position

	TIM2->CCMR1 |=  TIM_CCMR1_OC1M_2 ;
	TIM2->CCMR1 |=  TIM_CCMR1_OC1M_1 ;
	TIM2->CCMR1 &= ~TIM_CCMR1_OC1M_0 ;

	TIM2->CCER |= TIM_CCER_CC1E;

	//TIM2->CR1 |= TIM_CR1_DIR;

	//TIM2->CR1 |=TIM_CR1_CEN;
}


void SpiInit(void)
{
	RCC->APB2ENR|=RCC_APB2ENR_SPI1EN;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;

	GPIOA->AFR[0]|=GPIO_AFRL_AFRL7_0;
	GPIOA->AFR[0]|=GPIO_AFRL_AFRL7_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_1;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_3;

	GPIOA->AFR[0]|=GPIO_AFRL_AFRL6_0;
	GPIOA->AFR[0]|=GPIO_AFRL_AFRL6_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL6_1;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL6_3;

	GPIOA->AFR[0]|=GPIO_AFRL_AFRL5_0;
	GPIOA->AFR[0]|=GPIO_AFRL_AFRL5_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_1;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_3;

	GPIOA->MODER|=GPIO_MODER_MODER7_1 ;
	GPIOA->MODER|=GPIO_MODER_MODER6_1 ;
	GPIOA->PUPDR|=GPIO_PUPDR_PUPD6_0 ;
	GPIOA->MODER|=GPIO_MODER_MODER5_1 ;

	GPIOA->OSPEEDR|=GPIO_OSPEEDR_OSPEED7|GPIO_OSPEEDR_OSPEED6|GPIO_OSPEEDR_OSPEED5;

	//___________________________________CS
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER|=GPIO_MODER_MODER3_0;
	GPIOE->PUPDR|=GPIO_PUPDR_PUPD3_0;
	 GPIOE->BSRR|=GPIO_BSRR_BS3;                        //CS high



	//Boud Rate
	SPI1->CR1 &=~SPI_CR1_BR_0;
	SPI1->CR1 &=~SPI_CR1_BR_1;
	SPI1->CR1 |=SPI_CR1_BR_2;

	//Polarity
	SPI1->CR1 |=SPI_CR1_CPOL;    //0=0
	SPI1->CR1 |=SPI_CR1_CPHA;     //1=2

	//Bit 8/16
	SPI1->CR1|=SPI_CR1_DFF;       //1=16

	//frame format
	SPI1->CR1&=~SPI_CR1_LSBFIRST; //0-MSB

	//SSM SSI
	SPI1->CR1|=SPI_CR1_SSM;
	SPI1->CR1|=SPI_CR1_SSI;

	//interrupt
	//SPI1->CR2|=SPI_CR2_TXEIE;  //TX interrupt
	SPI1->CR2|=SPI_CR2_RXNEIE;  //RX interrupt


	//SPIE
	SPI1->CR1|=SPI_CR1_MSTR;  //master mode
	SPI1->CR1|=SPI_CR1_SPE;


	NVIC_EnableIRQ(SPI1_IRQn);       //vector interrupt
}

//*************************************************USART2 INIT*************************//
void UsartInit(void)
{

RCC->APB1ENR|=RCC_APB1ENR_USART2EN; //RCC USART
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;  //RCC GPIOA

                                    //RCC AF PA2-TX
GPIOA->AFR[0] |=  GPIO_AFRL_AFRL2_0;
GPIOA->AFR[0] |=  GPIO_AFRL_AFRL2_1;
GPIOA->AFR[0] |=  GPIO_AFRL_AFRL2_2;
GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL2_3;
									//RCC AF PA3-RX
GPIOA->AFR[0] |=  GPIO_AFRL_AFRL3_0;
GPIOA->AFR[0] |=  GPIO_AFRL_AFRL3_1;
GPIOA->AFR[0] |=  GPIO_AFRL_AFRL3_2;
GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL3_3;

GPIOA->MODER |=GPIO_MODER_MODER2_1;//AF
GPIOA->MODER |=GPIO_MODER_MODER3_1;//AF
GPIOA->PUPDR |=GPIO_PUPDR_PUPD3_0;

USART2->BRR = 0x1117;              //BR 9600

//USART2->CR1 |= USART_CR1_RXNEIE;   //Interrupt RX

USART2->CR1 &= ~USART_CR1_M;       //message length

USART2->CR1 |= USART_CR1_RE;	   //RE
USART2->CR1 |= USART_CR1_TE;	   //TE
USART2->CR1 |= USART_CR1_UE;       //UE

//NVIC_EnableIRQ(USART2_IRQn);       //vector interrupt
}




