#include "stm32f10x.h"

void Ports_Initialize()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable PortB
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function
	GPIOB->CRH |= ( (1<<4) | (1<<5) );   //PortB Pin9 Set to Output (50Mhz)   
	
	GPIOB->CRH |= (1<<7);    //PortB Pin9 set to
	GPIOB->CRH &= ~(1<<6);   // Alternate Function Output Push-Pull
	
}

void Timer_Initialize()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer 4
	TIM4->CCER |= TIM_CCER_CC4E;   //Output Enable
	TIM4->CR1 |= TIM_CR1_ARPE;   //Shadow Registers
	
	//PWM SETUP:
	TIM4->CCMR2 |=TIM_CCMR2_OC4M_1;   //Set to PWM Mode 1
	TIM4->CCMR2 |=TIM_CCMR2_OC4M_2;   //Set to PWM Mode 1
	
	TIM4->CCMR2 |=TIM_CCMR2_OC4PE;   //Pre-Load Enable
	
	TIM4->PSC = 32;
	TIM4->ARR = 1000;
	TIM4->CCR4 = 250;   //Comment Line 
	
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CR1 |= TIM_CR1_CEN;
}

int main()
{
	Ports_Initialize();
	Timer_Initialize();
	uint16_t i = 0;
	while(1)
	{
		i = 0;
		while(i<1000)
		{
			TIM4->CCR4 = i;
			i++;
		}
		
	}
	
}
