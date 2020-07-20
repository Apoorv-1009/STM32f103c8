//This Code is for sending pwm values, read from a joystick, to another STM32f103 via UART
//Rx
#include "stm32f10x.h"
volatile int myTicks=0;

void SysTick_Handler(void)
{
	myTicks++;
}
void delay_ms(int ms)
{
	myTicks=0;
	while(myTicks<ms);
}
void makeItGoFast(void)   //Sets Clock to 28Mhz
{
 //turn on external crystal
 RCC->CR |= RCC_CR_HSEON;

 //wait for HSE crystal be stable
 
 while(!(RCC->CR & RCC_CR_HSERDY));
 
 FLASH->ACR |= FLASH_ACR_PRFTBE;

 // Flash 2 wait state 
 FLASH->ACR &= ~(FLASH_ACR_LATENCY);  
 FLASH->ACR |= (uint32_t)0x1;    

 
 //configure RCC and PLL settings while PLL is off
 RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |  RCC_CFGR_PLLMULL);  //reset
 
 RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE);   //PLLXTPRE bit set to 0
 RCC->CFGR |= RCC_CFGR_PLLSRC;   //PLL source
 RCC->CFGR |= RCC_CFGR_PLLMULL7;  //PLL miultiplier 
 RCC->CFGR |= RCC_CFGR_HPRE_DIV2;  //AHB prescaler
 RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;  //APB1 presacaler 
 RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;  //APB2 prescaler
 
  
 //turn on PLL
 RCC->CR |= RCC_CR_PLLON; 
 while (!(RCC->CR & RCC_CR_PLLRDY)) ;
 
 //set PLL as clock source
 RCC->CFGR &= ~(RCC_CFGR_SW);  
 RCC->CFGR |= RCC_CFGR_SW_PLL;
 while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) ;

 SystemCoreClockUpdate();
 
}
void UART_Initialize(void)
{
	
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 28Mhz
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //PortA Clock Enable
	USART1->BRR |= 0x0B64; 			//Gives 9600 Baud Rate
	USART1->CR1 |= USART_CR1_RE | USART_CR1_UE;
	
		//Setting up PA10 (Rx) as Input:
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //Input
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_1);   //Floating Input
	GPIOA->CRH |= GPIO_CRH_CNF10_0;   
	/*
	!NOTE!: Input, Floating State is the default setting when
	you start your mcu, so setting up PA10 is optional.
	*/
}

void PWM_Initialize(void)
{
	//SET UP PB9:
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable PortB
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function
	GPIOB->CRH &= ~(GPIO_CRH_CNF9_0);   //Output 
	GPIOB->CRH |= GPIO_CRH_CNF9_1;   //Push-Pull
	GPIOB->CRH |= GPIO_CRH_MODE9_0;   //Output Mode
	GPIOB->CRH &= ~(GPIO_CRH_MODE9_1);   //10Mhz Max
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer 4
	TIM4->CCER |= TIM_CCER_CC4E;   //Output Enable
	TIM4->CR1 |= TIM_CR1_ARPE;   //Shadow Registers
	
	//PWM SETUP:
	TIM4->CCMR2 |=TIM_CCMR2_OC4M_1;   //Set to PWM Mode 1
	TIM4->CCMR2 |=TIM_CCMR2_OC4M_2;   //Set to PWM Mode 1
	
	TIM4->CCMR2 |=TIM_CCMR2_OC4PE;   //Pre-Load Enable
	
	TIM4->PSC = 1;
	TIM4->ARR = 255;
	TIM4->CCR4 = 0;   //Comment Line 
	
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CR1 |= TIM_CR1_CEN;
	
}


int main()
{
	makeItGoFast();
	
	//set up systick (1ms per interrupt)
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	
	UART_Initialize();
	PWM_Initialize();
	
while(1)
{
	if(USART1 ->SR & USART_SR_RXNE)
	{
	 TIM4->CCR4 = USART1->DR;
	}
}

}
