//This Code is for sending pwm values, read from a joystick, to another STM32f103 via UART
//Tx
#include "stm32f10x.h"
volatile int myTicks=0;
volatile int val=0;

void delay_ms(int ms)
{
	myTicks=0;
	while(myTicks<ms);
}
void SysTick_Handler(void)
{
	myTicks++;
}
void makeItGoFast(void)   //Sets Clock to 28Mhz
{
 //turn on external crystal
 RCC->CR |= RCC_CR_HSEON;

 //wait for HSE crystal be stable
 while(!(RCC->CR & RCC_CR_HSERDY))
  ;
 FLASH->ACR |= FLASH_ACR_PRFTBE;

 // Flash 2 wait state 
 FLASH->ACR &= ~(FLASH_ACR_LATENCY);
 FLASH->ACR |= (uint32_t)0x1;    

 //configure RCC and PLL settings while PLL is off
 RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |  RCC_CFGR_PLLMULL);  //reset
 
 RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE);   //PLLXTPRE bit set to 0
 RCC->CFGR |= RCC_CFGR_PLLSRC;   //pll source
 RCC->CFGR |= RCC_CFGR_PLLMULL7;  //pll miultiplier 
 RCC->CFGR |= RCC_CFGR_HPRE_DIV2;  //AHB prescaler
 RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;  //APB1 presacaler 
 RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;  //APB2 prescaler
  
 //turn on PLL
 RCC->CR |= RCC_CR_PLLON; 
 while (!(RCC->CR & RCC_CR_PLLRDY)) ;
 
 //set pll as clock source
 RCC->CFGR &= ~(RCC_CFGR_SW);
 RCC->CFGR |= RCC_CFGR_SW_PLL;
 while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) ;

 SystemCoreClockUpdate();
 
}
void UART_Initalize(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 28Mhz
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //PortA Clock Enable
	
	/*
	REMAPPING:
	By Default, UART1 is set on PA9 AND PA10, if we wish to
	move it to PB6 AND PB7, we need to remap it.
	If we remap it, remember to enable PortB Clock
	Page 175 of the Reference Manual talks about remaping
	Refer to the AFIO Section of the Reference Manual for 
	more details
	AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
	*/
	
	//Setting up PA9 (Tx) as Output: 
	GPIOA->CRH |= GPIO_CRH_MODE9;   //Output (50Mhz)
	GPIOA->CRH |= GPIO_CRH_CNF9_1;   //Alternate Function
	GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);   //Output Push-Pull
	
	//Setting up Baud Rate:
	USART1->BRR |= 0x0B64;   //Gives 9600 Baud Rate
	
	//              Tx Enable     UART Enable
	USART1->CR1 |= USART_CR1_TE | USART_CR1_UE;
	
}
void ADC_Initialize(void)
{
	//CONFIGURE PA5:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //Port A Enable
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function
	//Input Push-Pull
	GPIOA->CRL |= GPIO_CRL_CNF5_1;   
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_0);
	GPIOA->CRL &= ~(GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1);   //Input Mode
	
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

void ADC1_2_IRQHandler(void)
{
	//Get value at the End Of conversion, automatically clears EOC interrupt bit
	val = (ADC1->DR)/16;   //Convert to 8 Bit data
	USART1->DR = val;
	while(!(USART1 ->SR & USART_SR_TXE));
}

int main()
{
	//SET UP TIMER FOR DELAYS:
	makeItGoFast();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	UART_Initalize();
	ADC_Initialize();

	while(1)
	{
		//Nothing To put in while as all data is sent thru the ADC1_2_IRQHandler function
	}	

}
