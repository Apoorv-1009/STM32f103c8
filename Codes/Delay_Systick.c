#include "stm32f10x.h"
volatile int myTicks = 0;

void Port_Initialize()
{
	RCC->APB2ENR |= (1<<4);   // Set clock for Port C
	GPIOC->CRH |= ( (1<<20) | (1<<21) );   //Output (50 Mhz) 
	GPIOC->CRH &= ~( (1<<22) | (1<<23) );   //General Purpose Output
}

void SysTick_Initialize()   
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
}

void SysTick_Handler(void)
{
	myTicks++;
}

void delay_ms(int ms)
{
	myTicks = 0;
	while(myTicks<ms);
}
int main()
{
	int del = 1000;   //Enter Delay Value here
	Port_Initialize();
	SysTick_Initialize();
	
	while(1)
	{
		GPIOC->BSRR = (1<<13);
		delay_ms(del);
		GPIOC->BSRR = (1<<13+16);
		delay_ms(del);
	}
}