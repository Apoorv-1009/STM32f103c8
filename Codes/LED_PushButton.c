//This Code is to turn on an LED when a button is pushed
#include "stm32f10x.h"

void Ports_Intialize(void)
{
	//SET-UP PC13(Onboard Led):
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;   //Port C Clock Enable
	GPIOC->CRH |= GPIO_CRH_MODE13;   //Output (Max f 50Mhz)
	GPIOC->CRH &= ~(GPIO_CRH_CNF13_1 | GPIO_CRH_CNF13_0);   //General Purpose Output Pushpull
	
	//SET-UP PB5(Button):
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Port B Clock Enable
	GPIOB->CRL &= ~(GPIO_CRL_MODE5);   //Input Mode
	//Input with Pull-Up, Pull-Down
	GPIOB->CRL |= (GPIO_CRL_CNF5_1);   
	GPIOB->CRL &= ~(GPIO_CRL_CNF5_0);
}

int main()
{
	
	Ports_Initialize();
	int pressed = 0;

	while(1)
	{
		//Also Try:
		//pressed = GPIOB->IDR & 0b0000000000000101;
		pressed = GPIOB->IDR & GPIO_IDR_IDR5;
		
		if(pressed)
		{
			GPIOC->BSRR |= (1 << 13);
			//GPIOC->BSRR &= (0 << (13+16));
		}
		
		else
		{
			GPIOC->BSRR |= (1 << (13+16));
			//GPIOC->BSRR &= (0 << 13);
		}
	}
	
}
