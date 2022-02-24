#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"

uint16_t adc_Voltage;
void TIM3_IRQHandler(void);

int main()
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		
	
	// configure timer3 for change color after 20s
	TIM3 -> SR = 0x0;
	// becuase micro clock frequency = 24 Mhz so we set 
	// prescaler = 10000-1 and auto-reload = 48000-1
	TIM3 -> PSC = 10000-1;
	TIM3 -> ARR = 48000-1;
	TIM3 -> CR1 &= ~TIM_CR1_DIR; //upcouner timer
	
	// enable nvic:
	NVIC_SetPriority(TIM3_IRQn, 1);
	NVIC_EnableIRQ(TIM3_IRQn);
	
	// turn on timer3 clock:
	TIM3 -> CR1 |= TIM_CR1_URS; // just overflow generate interrupt
	TIM3 -> CR1 |= TIM_CR1_CEN;
	TIM3 -> DIER |= TIM_DIER_UIE; // update interrupt enable;
	
	
	// configure timer2 for having pwm
	TIM2 -> SR = 0x0;
	TIM2 -> PSC = 2400;
	TIM2 -> ARR = 1000;
	TIM2 -> CR1 &= ~TIM_CR1_DIR; //upcouner timer
	
	
	// set channel 4 of timer2 as output:
	TIM2 -> CCMR2 &= ~(TIM_CCMR2_CC4S);
	
	// enable pwm mode1 for channel 4 for timer2(by write 110 to OCXM):
	TIM2 -> CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
	
	// enable preloads for channels:
	TIM2 -> CCMR2 |= (TIM_CCMR2_OC4PE);
	
	// set ARPE = 1 for timer2:
	TIM2 -> CR1 |= (TIM_CR1_ARPE);
	TIM2 -> CCR4 = 0x0032;
	
	// turn on pwm channels:
	TIM2 -> CCER |= (TIM_CCER_CC4E);
	
	TIM2 -> CR1 |= (TIM_CR1_URS | TIM_CR1_CEN);
	TIM2 -> DIER &= ~TIM_DIER_UIE; //update interrupt disable;
	
	// Configure the GPIOs
	//GPIO_Configuration
	GPIO_InitTypeDef GPIO_InitStructure1, GPIO_InitStructure2;

	// Configure USART3 Tx (PB.10) and Rx (PB.11) as alternate function push-pull //
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure1);
	
	// Configure ADC (PA.02) and PWM (PA.03) as alternate function push-pull //
	GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure2);

	// Configure the USART3 
	//USART_Configuration
	USART_InitTypeDef USART_InitStructure;

	// USART3 configuration
	/* USART3 configured as follow:
	   - BaudRate = 115200 baud
	   - Word Length = 8 Bits
	   - One Stop Bit
	   - No parity
	   - Hardware flow control disabled (RTS and CTS signals)
	   - Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	// Enable USART2
	USART_Cmd(USART3, ENABLE);
	
	// turn on ADC
	ADC1->CR2 |= ADC_CR2_ADON;
	while(!(ADC1->CR2 & ADC_CR2_ADON));
	
	// ADC calibration
	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL);
	
	// continuos
	ADC1->CR2 |= ADC_CR2_CONT;	
	
	// sample time
	ADC1->SMPR2 &= ~(ADC_SMPR2_SMP2);
	
	// first conversion and second pin
	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->SQR3 |= 0x00000002;
	
	// start converstion
	ADC1->CR2 |= ADC_CR2_ADON;
	

	while(1)
	{
		if (ADC1->SR & ADC_SR_EOC)
		{
			adc_Voltage = ADC1->DR & ADC_DR_DATA;
			ADC1->SR &= (uint16_t) ~(ADC_SR_EOC);
			ADC1->CR2 |= ADC_CR2_ADON;
			USART_SendData(USART3, adc_Voltage);
		}
	}
}

void TIM3_IRQHandler(void){
	// clear interrupt flag from status register:
	TIM3 -> SR &= ~TIM_SR_UIF;
	
	uint16_t current = TIM2 -> CCR4;
	if (current != 950){
		TIM2 -> CCR4 = current + 150;
	}
	else
		TIM2 -> CCR4 = 50;
}
