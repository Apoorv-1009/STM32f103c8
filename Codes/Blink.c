#include "stm32f10x.h"

void Initialize()
{
	RCC->APB2ENR |= (1<<4);   // Set clock for Port C
	GPIOC->CRH |= ( (1<<20) | (1<<21) );   //Output (50 Mhz) 
	GPIOC->CRH &= ~( (1<<22) | (1<<23) );   //General Purpose Output
}

void delay(int k)
{
	int i = 0;
	while(i<k)
		i++;
}
int main()
{
	Initialize();
	while(1)
	{
		GPIOC->BSRR = (1<<13);   //Set Pin13 High
		delay(2000000);   //Delay for 1 second
		GPIOC->BSRR = (1<<13 + 16);   //Set Pin13 Low
		delay(2000000);   //Delay for 1 second
	}
	
	
}