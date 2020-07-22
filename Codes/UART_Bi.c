//This Code is to Blink an LED on one ST32f103c8 when a button is pressed on another STM32f103c8
#include "stm32f10x.h"

volatile uint16_t myTicks = 0;

void delay_ms(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}

void Blink(int n)
{
	//Blink LED on PC13
	int i = 0;
	while(i < n)
	{
		GPIOC->BSRR |= (1<<13);
		delay_ms(500);
		GPIOC->BSRR |= (1<<(13+16));
		delay_ms(500);
		i++;
	}
}
void UART_Write(unsigned char data)
{
	USART1->DR = data;
	while(!(USART1->SR & USART_SR_TC));   //Check Status Register if Transmisson is complete
}

void UART_Read()
{
	volatile uint16_t val = 0;
	while(!(USART1 ->SR & USART_SR_RXNE));   //Check Status Register if all is Recieved 
	val = USART1->DR;
	if(val == 42)
		Blink(3);

}

void UART_Initialize()
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 72Mhz
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //PortA Clock Enable
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Alternate Function Enable
	
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
	
	//Setting up PA10 (Rx) as Input:
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //Input
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_1);   //Floating Input
	GPIOA->CRH |= GPIO_CRH_CNF10_0;   
	/*
	!NOTE!: Input, Floating State is the default setting when
	you start your mcu, so setting up PA10 is optional.
	*/
	
	//Setting up Baud Rate:
	USART1->BRR |= 0x0B64; 			//Gives 9600 Baud Rate
	
	//              Rx enable      Tx Enable     UART Enable
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}


void Ports_Initialize()
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

void SysTick_Initialize()   
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
}


void SysTick_Handler(void)
{
	myTicks++;
}

int main()
{
	SysTick_Initialize();
	UART_Initialize();
	Ports_Initialize();
	int pressed = 0;
	
	while(1)
	{
		pressed = (GPIOB->IDR & GPIO_IDR_IDR5);
		
		if(pressed)
			UART_Write(42);
		
		else
			UART_Write(69);
		
		UART_Read();
		
	}
	
}

