/*
 Code implements Motor Code on LEDs thru PWM with Timer4, by reading Joystick values
 Setup:
 PWM LEDs: PB6(Ch1), PB7(Ch2)
 Joystick: PA1(Ch1), PA2(Ch2)
 x->PA1   y->PA2
       Left           Right
 LED:  PA4            PA5
 PWM:  PB6            PB7
 */

#include "stm32f1xx.h"
#include "stdlib.h"
#include "main.h"

volatile int myTicks = 0;
volatile uint16_t samples[2] ={ 0, 0 };
//x -> samples[0]
//y -> samples[1]

void GPIO_Initialize()
{
	//Enable Clocks:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //Enable Clock for Port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable Clock for Port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function

	//Setup PA1:
	GPIOA->CRL &= ~(GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);   //INPUT Mode
	GPIOA->CRL &= ~(GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1);   //Input Analog

	//Setup PA2:
	GPIOA->CRL &= ~(GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1);   //INPUT Mode
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1);   //Input Analog

	//Setup PA4:
	GPIOA->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull

	//Setup PA5:
	GPIOA->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull

	//Setup PB6:
	GPIOB->CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	//Setup PB7:
	GPIOB->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

}

void Timer_Initialize()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer4
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //Enable Channel 1 and 2 as OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2

	//PWM Mode 1 for Channel 1:
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1; //freq/1 = 8 Mhz
	TIM4->ARR = 4095;   //16 Bit value
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void ADC_Initialize()
{
	GPIOA->BSRR |= 1 << 4 | 1 << 5;
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;   //f/6 = 72/6 = 12 Mhz
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   //Enable ADC1 Clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;   //Enable DMA1 Clock

	ADC1->SMPR2 |= ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0;   //601.5 Sampling Rate for Channel 1
	ADC1->SMPR2 |= ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0;   //601.5 Sampling Rate for Channel 2

	ADC1->SQR1 |= 1 << 20;   //Set length of 2 ADC conversions
	//Set channel you want to convert in the sequence registers:
	ADC1->SQR3 |= ADC_SQR3_SQ1_0;   //Channel 1, Sequence 1
	ADC1->SQR3 |= ADC_SQR3_SQ2_1;	  //Channel 2, Sequence 2

	ADC1->CR1 |= ADC_CR1_SCAN;   //Enable Scan Mode
	ADC1->CR2 |= ADC_CR2_DMA;   //Enable DMA Mode

	//DMA Settings:
	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));   //Peripheral to READ from
	DMA1_Channel1->CMAR = (uint32_t) samples;   //Base Address of memory to WRITE to
	DMA1_Channel1->CNDTR = 2;   //Define number of times to transfer data

	DMA1_Channel1->CCR |= DMA_CCR_CIRC;   //Enable Circular Mode
	DMA1_Channel1->CCR |= DMA_CCR_MINC;   //Enable Memory Increment Mode
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;   //Define Peripheral Data size as 16 bits
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;   //Define Memory size as 16 bits

	DMA1_Channel1->CCR |= DMA_CCR_EN;   //Enable DMA1
	//!NOTE! Do NOT Enable DMA before you define all of its settings

	ADC1->CR2 |= ADC_CR2_ADON;   //Enable ADC for the 1st time
	ADC1->CR2 |= ADC_CR2_CONT;   //Set ADC to Continuous Mode
	//delay_ms(1);

	//Turn on ADC for the 2nd time to acually turn it on:
	ADC1->CR2 |= ADC_CR2_ADON;
	//delay_ms(1);

	//Run Calibration:
	ADC1->CR2 |= ADC_CR2_CAL;
	//delay_ms(2);
}

void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y)
{
	if (DL == 1)
		GPIOA->BSRR |= 1 << 4;   //Turn on LEFT LED
	else
		GPIOA->BRR |= 1 << 4;   //Turn off LEFT LED

	if (DR == 1)
		GPIOA->BSRR |= 1 << 5;   //Turn on RIGHT LED
	else
		GPIOA->BRR |= 1 << 5;   //Turn off RIGHT LED

	TIM4->CCR1 = (uint32_t) abs(abs(a*X) - abs(b*Y));   //Left PWM
	TIM4->CCR2 = (uint32_t) abs(abs(p*X) - abs(q*Y));   //Right PWM
}

int mapp(float k, float l, float h, float L, float H)
{
	return ((k - l) / (h - l)) * (H - L) + L;
}

int read(int k)
{
	int val = 0;
	val = samples[k];
	if(k == 0)
		val = mapp(val, 0, 4095, -4095, 4095);

	else
	{
		val = mapp(val, 0, 4095, 4095, -4095);
	}

	if (abs(val) < 100)
		val = 0;
	
	if (val < -3900)
		val = -4095;

	if (val > 3900)
		val = 4095;
	/**/
	return val;
}

void MotorCode(int x, int y)
{
//void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y)

	if (abs(x) < 20 && abs(y) < 20)   //No Motion
		Drive(0,0,0,0,0,0,0,0);

	else if(abs(x) < 10 && y < 0)   //Full Backward
		Drive(0,0,0,1,0,1,x,y);

	else if(abs(x) < 10 && y > 0)   //Full Forward
		Drive(1,1,0,1,0,1,x,y);

	else if (x < 0 && abs(y) <= 10)   //Spot Turn Left
			Drive(0,1,1,0,1,0,x,y);

	else if (x > 0 && abs(y) <= 10)   //Spot Turn Right
			Drive(1,0,1,0,1,0,x,y);

	else if(x > 0 && y > 0 && x > y)   //Octet 1
		Drive(1,0,1,0,1,1,x,y);

	else if(x > 0 && y > 0 && x < y)   //Octet 2
		Drive(1,1,0,1,1,1,x,y);

	else if(x < 0 && y > 0 && abs(x) < y)   //Octet 3
		Drive(1,1,1,1,0,1,x,y);

	else if(x < 0 && y > 0 && abs(x) >= y)   //Octet 4
		Drive(0,1,1,1,1,0,x,y);
	/**/

	else if(x < 0 && y < 0 && abs(x) > abs(y))   //Octet 5
	 	 Drive(0,1,1,0,1,1,x,y);

	else if(x < 0 && y < 0 && abs(x) < abs(y))   //Octet 6
	 	 Drive(0,0,0,1,1,1,x,y);

	else if(x > 0 && y < 0 && abs(x) < abs(y))   //Octet 7
	 	 Drive(0,0,1,1,0,1,x,y);

	else if(x > 0 && y < 0 && abs(x) > abs(y))   //Octet 8
	 	 Drive(0,1,1,0,1,1,x,y);

	//Test Drive:
	//Drive(1,1,1,0,0,1,x,y);
}

int main()
{
	//SysTick_Initialize();
	GPIO_Initialize();
	Timer_Initialize();
	ADC_Initialize();

	int x_read, y_read;
	while (1)
	{
		x_read = read(0);   //Read mapped x co-ordinate of Joystick
		y_read = read(1);   //Read mapped y co-ordinate of Joystick


		MotorCode(x_read, y_read);
	}
}

