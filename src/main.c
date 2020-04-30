#include <stdint.h>		
#include "stm32f4xx.h"
#include "lcd1602.h"

#define frequency	16000000UL	// 16 MHz high-speed internal (RC)

/*	GLOBAL VARIABLES START				*/

uint8_t LCD_show_ready = 0, debounce_ms_enable = 0, pressure_flag;
uint8_t in_X = 1, in_Y = 1, in_mode = 0;	// ADD by default
uint16_t pressure_10_ms_ticks = 0;

/*	GLOBAL VARIABLES END				*/

/************************************************** SYSTICK **************************************************/

void init_systick(void)
{
	SysTick->CTRL = 0;						// disable SysTick
	SysTick->LOAD = frequency - 1;					// set the initial reload value
	SysTick->VAL = 0;						// reset the curent SysTick counter value
	
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	// switch off

	// use proc.clock (1 = processor clock, 0 = external clock (HCLK/8 = 2 MHz)), enable interrupts (1 = enable)
}

void SysTick_Handler(void)
{
	if(debounce_ms_enable)
	{
		EXTI->IMR |= (EXTI_IMR_IM13 | EXTI_IMR_IM14 | EXTI_IMR_IM15);	// enable disabled interrupts
		
		debounce_ms_enable = 0;
		
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  	
	}
}

void systick_debounce_ms(uint32_t debounce_ms)
{
	// default value is 250 ms debounce
	uint32_t time_ms = (debounce_ms >= 1 && debounce_ms <= 1000) ? debounce_ms : 250;	
	
	SysTick->VAL = 0;					// clear current CNT value
	SysTick->LOAD = ((frequency / 1000) * time_ms) - 1;	// (time * 10^-3 * frequency) - 1 = (time * 10^-3 * 16 000 000) - 1 = (time * 16 000) - 1

	debounce_ms_enable = 1;
	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	// enable SysTick
}

/**********************************************************************************************************/

/************************************************** TIM2 **************************************************/
void tim2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;				// TIM2 clock
	TIM2->PSC = 16000 - 1;						// 1000 ticks per second - 1ms - prescaler 
	TIM2->ARR = 10-1;						// T = 10 ms = 10 ticks -> {0, 1, 2, 3, 4, 5, 6, 7, 8, 9} - auto-reload-register
	TIM2->DIER |= TIM_DIER_UIE;					// TIM2 DMA/interrupt enable register - allow events by timer -  Update interrupt enable 		
	TIM2->CR1 &= ~TIM_CR1_DIR;					// upcounting, DIR = 0
		
	NVIC_EnableIRQ(TIM2_IRQn);	// enable interrupts by TIM2
	__enable_irq();	// global interrupts enable
}

void TIM2_IRQHandler(void)
{
	TIM2->SR &= ~TIM_SR_UIF;	// clear pending bit in the interrupt handler
			
	pressure_10_ms_ticks = ((GPIOB->IDR & GPIO_IDR_ID15) == pressure_flag) ? (pressure_10_ms_ticks + 1) : 0;
	
	if(pressure_10_ms_ticks >= 200)	// 200 x 10ms = 2000ms = 2s --> set default values
	{
		in_X = 1;	
		in_Y = 1;
		in_mode = 0;	
		
		LCD_show_ready = 1;
		
		pressure_10_ms_ticks = 0;	// reset cnt 
		
		TIM2->CR1 &= ~TIM_CR1_CEN;	// switch off TIM2
	}
}

void btn_pressure_check(void)
{
	// set pressure flag:
	pressure_flag = (GPIOB->IDR & GPIO_IDR_ID15);	// PB15 == 1 --> pressure_flag = 1
	
	TIM2->CR1 |= TIM_CR1_CEN;			// switch on TIM2
}

/**********************************************************************************************************/

/************************************************** GPIO **************************************************/

void init_GPIO(void)
{
	// OSPEEDR - 2 MHz by default after reset
	
	// RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// enable GPIOA
	// GPIOA->MODER |= GPIO_MODER_MODE5_0;	// PA5 for output - Nucleo onboard LED
	// GPIOA->MODER &= ~GPIO_MODER_MODE5_1;	// push-pull mode is by default
	
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// enable GPIOB
	
	GPIOB->MODER &= ~GPIO_MODER_MODE13;			// PB13 for input 
	GPIOB->MODER &= ~GPIO_MODER_MODE14;			// PB14 for input
	GPIOB->MODER &= ~GPIO_MODER_MODE15;			// PB15 for input
	
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD13_0;			// pull-up for PB13 - [01]
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD13_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD14_0;			// pull-up for PB14 - [01]
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD14_1;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD15_0;			// pull-up for PB15 - [01]
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD15_1;
}

/**********************************************************************************************************/

/************************************************** EXTI **************************************************/

void btn_irq_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	// SYSCFG (System configuration controller) clock through APB2 bus enable
	
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PB;	// interrupt source selection, PB13 -> EXTI13
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PB;	// interrupt source selection, PB14 -> EXTI14
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PB;	// PB15 -> EXTI15
	
	EXTI->IMR |= (EXTI_IMR_IM13 | EXTI_IMR_IM14 | EXTI_IMR_IM15);	// interrupt request mask - IM11 and IM12 are not masked now
	EXTI->FTSR |= (EXTI_FTSR_TR13 | EXTI_FTSR_TR14 | EXTI_FTSR_TR15);	// falling trigger
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 0);	// highest priority
	
	__enable_irq();	// enable interrupts, PRIMASK reset
}

void EXTI15_10_IRQHandler(void)
{
	EXTI->IMR &= ~EXTI_IMR_IM13;	// disable EXTI13 interrupts
	EXTI->IMR &= ~EXTI_IMR_IM14;	// disable EXTI14 interrupts
	EXTI->IMR &= ~EXTI_IMR_IM15;	// disable EXTI15 interrupts
	
	systick_debounce_ms(500);	// start debouncing 
	
	if(EXTI->PR & EXTI_PR_PR13)	// PB13 - in_X
	{
		in_X++;
						
		EXTI->PR |= EXTI_PR_PR13;	// clear pending flag by writing 1 (clear event flag after work)
	}
	else if(EXTI->PR & EXTI_PR_PR14)	// PB14 - in_Y
	{
		in_Y++;

		EXTI->PR |= EXTI_PR_PR14;	// clear pending flag by writing 1 	
	}
	else if(EXTI->PR & EXTI_PR_PR15)	// PB15 - MODE/RST
	{
		btn_pressure_check();		// save PB15 IDR value --> start TIM2 10ms counting for 3 seconds
		
		in_mode = (in_mode <= 2) ? (in_mode + 1) : 0;
		
		EXTI->PR |= EXTI_PR_PR15;	// clear pending flag by writing 1
	}
	
	LCD_show_ready = 1;
}

/*********************************************************************************************************/

/************************************************** ASM **************************************************/

__asm uint32_t arithmetic_function(uint8_t X, uint8_t Y, uint8_t mode) 
{
	// embedded assembly function	
	
	// X - RO, Y - R1, mode - R2
	
	// mode == 0 --> Z = (A + B) 
	// mode == 1 --> Z = (A - B) 
	// mode == 2 --> Z = (A * B) 
	// mode == 3 --> Z = (A / B) 
	
	CMP	R2, #0		// mode == 1?
	BEQ	add_label 
	
	CMP	R2, #1		// mode == 2?
	BEQ	sub_label
	
	CMP	R2, #2		// mode == 3?
	BEQ	mul_label
	
	CMP	R2, #3		// mode == 4?
	BEQ	div_label
	
add_label	
	ADD	R0, R0, R1	// R0 = R0 + R1 = X + Y
	BAL	end_label
	
sub_label
	SUB	R0, R0, R1	// R0 = R0 - R1 = X - Y
	BAL	end_label
	
mul_label
	MUL	R0, R0, R1	// R0 = R0 * R1 = X * Y
	BAL	end_label
	
div_label
	UDIV	R0, R0, R1	// R0 = R0 / R1 = X / Y
	BAL	end_label
	
end_label
	BX 	LR
}

/***************************************************************************************************************/

/************************************************** main loop **************************************************/

int main(void)
{	
	init_GPIO();
	btn_irq_init();
	init_systick();
	tim2_init();
	
	I2C1_init();
	LCD1602_init();	

	while(1)
	{		
		if(LCD_show_ready)
		{
			arith_display(in_X, in_Y, in_mode);
			
			LCD_show_ready = 0;
		}
		else
		{
			__NOP;
		}
	}
}
