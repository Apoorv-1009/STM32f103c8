#include "stm32f10x.h"

int myTicks = 0;

void Ports_Initialize()   //Ports Initialize
{
	RCC->APB2ENR |= (1<<4);   // Set clock for Port C
	GPIOC->CRH |= ( (1<<20) | (1<<21) );   //Output (50 Mhz) 
	GPIOC->CRH &= ( (0<<22) | (0<<23) );   //General Purpose Output
}

void Timer_Initialize()   //Timer Initialize
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable TIM4 Clock Gating
	TIM4->PSC = 1;
	TIM4->ARR = 72;   //Implies 1Mhz Frequency
	TIM4->CR1 |= TIM_CR1_URS;   //Only counter overflow/underflow generates an interrupt
	TIM4->DIER |=TIM_DIER_UIE;   //UG: Update Generation
	NVIC_EnableIRQ(TIM4_IRQn);   //Abort Enable IRQ Line for TIM4
}
	
void TIM4_IRQHandler(void)
{
	myTicks++;
	TIM4->SR &= ~TIM_SR_UIF;   //Update Interrupt To Reset
}
	

void delay_ms(int ms)
{
	TIM4->CR1 |= TIM_CR1_CEN;   //Enable Timer4
	myTicks = 0;
	ms = ms*1000;
	while(myTicks < ms);
	TIM4->CR1 &= ~TIM_CR1_CEN;   //Disable TIM4
}

int main()
{
	Ports_Initialize();
	Timer_Initialize();
	int del = 1000;   //Enter Delay Value here
	while(1)
	{
			GPIOC->BSRR = (1<<13);   //Set Pin13 High
			delay_ms(del);
			GPIOC->BSRR = (1<<13+16);   //Set Pin13 Low
			delay_ms(del);
	}
	
}

