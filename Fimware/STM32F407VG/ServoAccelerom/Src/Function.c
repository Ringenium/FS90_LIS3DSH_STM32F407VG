


//*********************** USE FUNCTION*******************//



#include "stm32f4xx.h"
#include "Function.h"





//*********************LISt of FUNCTION**************//

//************SPI1 to LIS3DSH*************//


void DatToLIS(uint16_t message)
{
	GPIOE->BSRR|=GPIO_BSRR_BR3;                       //CS low

			  while(SPI1->SR & SPI_SR_TXE)
			  SPI1->DR=message;

			  while(SPI1->SR & SPI_SR_BSY);
			  while(SPI1->SR & SPI_SR_RXNE);
			  	GPIOE->BSRR|=GPIO_BSRR_BS3;

}



//************STM32F4 to USART2 char transmit**********//

void Usart2ChTx(char chr)
{
	while(!(USART2->SR & USART_SR_TC));
	USART2->DR=chr;
}

//*******STM32F4 to USART2 string Data transmit*********//

void  Usart2DatTx(char* st)
{
	uint8_t i=0;
	while(st[i])
	{
		Usart2ChTx(st[i++]);
	}


}

//************SIGN OF NUMBER + return mg************************//
/*
 * sign-часть числа отвечающая за знак
 *number-число целиком
 *factor-множитель меняющийся от диапазона измерения g см. даташит
 *err-поправка для задания нулевого положения
 */

int Convmg(char sign,int16_t number, float factor,uint8_t err)
{
	uint8_t FLst;
	uint16_t nnDataSPI;
	float ans;
	int Ans;

	FLst=sign & 0x80;

	switch (FLst){

	case 0x80:                    //подсчет отрицательного числа

		nnDataSPI=~number;
		Ans=nnDataSPI;
		ans=Ans*factor;
		Ans=err-ans;
		break;

	case 0x00:                    //подсчет положительного числа

		Ans=number;
		ans=Ans*0.06;
		Ans=ans;
		break;
				}
	return Ans;
}



