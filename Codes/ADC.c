#include "stm32f10x.h"

float val = 0;
volatile static uint32_t myTicks = 0;

void SysTick_Handler(void)
{
	myTicks++;
}

void delay_ms(uint32_t ms)
{
	myTicks = 0;
	while(myTicks < ms);
}

void ADC1_2_IRQHandler(void)
{
	
	//Check if we are here because end of conversion flag is set
	//The end of conversion flag is cleared by reading the data register
	if(ADC1->SR & ADC_SR_EOC)
		val = ADC1->DR;
}

uint16_t map(float k, float l, float h, float L, float H)
{
	return ((k - l)/(h - l))*(H - L) + L;
}

void Config(void)
{
	//CONFIGURE PA5:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //Port A Enable
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function
	//Input Push-Pull
	GPIOA->CRL |= GPIO_CRL_CNF5_1;   
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_0);
	GPIOA->CRL &= ~(GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1);   //Input Mode
	
	//CONFIGURE PB9:
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable PortB
	GPIOB->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0);   //Pb9 set to Output (Max 50Mhz)   
	GPIOB->CRH |= (GPIO_CRH_CNF9_1);   //Pb9 set to Alternate Function 
	GPIOB->CRH &= ~(GPIO_CRH_CNF9_0);   //Output Push-Pull
	
	//SET UP TIMER FOR DELAYS:
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
		//SET TIMERS FOR PWM:
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer 4
	TIM4->CCER |= TIM_CCER_CC4E;   //Output Enable
	TIM4->CR1 |= TIM_CR1_ARPE;   //Shadow Registers
	//Pwm setup:
	TIM4->CCMR2 |=TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;   //Set to PWM Mode 1
	
	TIM4->CCMR2 |=TIM_CCMR2_OC4PE;   //Pre-Load Enable
	
	TIM4->PSC = 32;
	TIM4->ARR = 1000;
	TIM4->CCR4 = 0;   //Comment Line 
	
	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Timer
	
	//SET UP ADC:
	//Set Prescaler to not excede 14Mhz
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;   //f/6 = 12Mhz (Given on line 1771 of the stm32f10x.h Header File 
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   //ADC1 Enable
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Alternate Function Enable   
	
	ADC1->CR1 |= ADC_CR1_EOCIE;   //Enable End Of Conversion Interrupt
	NVIC_EnableIRQ(ADC1_2_IRQn);   //Enable Interrupt in NVIC
	
	 //Set channel you want to convert  in the sequence registers
	ADC1->SQR3 |= ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2;   //5<<0
	
	//Set Sampling Rate as 239.5 Cycles/s 
	ADC1->SMPR2 |= ADC_SMPR2_SMP5_2 | ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_0;   
	
	//Enable ADC for 1st Run and set to continuous mode
	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT;
	
	delay_ms(1);   //Mandatory delay to turn on ADC
	
	//Turn on ADC for the 2nd time to actually turn it on
	ADC1->CR2 |= ADC_CR2_ADON;
	
	delay_ms(1);
	
	ADC1->CR2 |= ADC_CR2_CAL;   //Run Calibration
	while(ADC1->CR2 & ADC_CR2_CAL);   //The proper way to check for calibration*/
	delay_ms(5);
	
}


int main()
{
	uint16_t PWM = 0;
	Config();
	while(1)
	{
		PWM = map(val,0,4095,0,1000);
		TIM4->CCR4 = PWM;
	}
	
}

