//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// This is source code for the final lab project.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include <stdlib.h>
#include <math.h>
#include "stm32f0-stdperiph\stm32f0xx_spi.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

/* 
* GPIO Pin List *
-----------------
* PA0: ADC
* PA1: Frequency Input
* PA4: DAC
* PB3: SPI SCK
* PB4: Parallel LCK
* PB5: SPI MOSI
*/

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


// User defines
/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

#define LCD_LCK_PIN GPIO_Pin_4
#define LCD_DISABLE ((uint8_t)0x0)
#define LCD_ENABLE ((uint8_t)0x80)
#define CMD ((uint8_t)0x0)
#define DATA ((uint8_t)0x40)

//Global variables
int res = 0;
double freq = 0.0000;
double period = 0;
int edge_count = 0;


// Function definitions
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void mySPI_Init(void);
void myLCD_Init(void);
void myDelay_Init(void);
void myDelay(uint32_t milli);
void Send_LCD(uint8_t word, uint8_t type);
void write_LCD(uint8_t word);
void myADC_Init();
void myDAC_Init();
void move_LCD_cursor(uint8_t address);
void write_upper();
void write_lower();
void write_all();
uint16_t get_res();
void Set_DAC(uint16_t DAC_Raw);

int main(int argc, char* argv[]) {
	uint16_t D2A = 0;

	myGPIOA_Init(); /* Initialize I/O port PA */
	myTIM2_Init(); /* Initialize timer TIM2 */
	myEXTI_Init(); /* Initialize EXTI */
	mySPI_Init();
	myDelay_Init();
	myGPIOB_Init();
	myLCD_Init();
	myDAC_Init();
	myADC_Init();

	while (1) {
		D2A = get_res();
		Set_DAC(D2A);
		write_all();
	}
	return 0;
}
void mySPI_Init() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);
}

void myDelay_Init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 47999;
	TIM3->CR1 = ((uint16_t)0x8C);
	TIM3->ARR = 100;
	TIM3->EGR |= 0x1;
}

void myDelay(uint32_t milli){
	TIM3->CNT = 0x0;
	TIM3->ARR = milli;
	TIM3->EGR |=0x1;
	TIM3->CR1 |= TIM_CR1_CEN;
	while((TIM3->CR1 & TIM_CR1_CEN)!= 0 );
}

void myLCD_Init() {
	Send_LCD(0x2, CMD);
	myDelay(2);
	Send_LCD(0x28, CMD);
	myDelay(2);
	Send_LCD(0xC, CMD);
	myDelay(2);
	Send_LCD(0x6, CMD);
	myDelay(2);
	Send_LCD(0x1, CMD);
	myDelay(2);
       char upper[9] = {'F',':',' ',' ',' ',' ','H','z','\0'};
       char lower[9] = {'R',':',' ',' ',' ',' ',' O','h','\0'};


	for(int i=0; i < 9; i++){
		Send_LCD(upper[i],DATA);
	}
	move_LCD_cursor(0x40);
       for(int i=0; i < 9; i++){
		Send_LCD(lower[i],DATA);
	}
}

void Send_LCD(uint8_t word, uint8_t type){
	uint8_t high = ((word&0xF0)>>4) ;
	uint8_t low = word & 0x0F;

	write_LCD(LCD_DISABLE|high|type);
	write_LCD(LCD_ENABLE|high|type);
	write_LCD(LCD_DISABLE|high|type);

	write_LCD(LCD_DISABLE|low|type);
	write_LCD(LCD_ENABLE|low|type);
	write_LCD(LCD_DISABLE|low|type);
}

void write_LCD(uint8_t word){
	GPIOB->BRR |= LCD_LCK_PIN;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == 1);
		SPI_SendData8(SPI1,word);
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == 1);
	GPIOB->BSRR |= LCD_LCK_PIN;
}

void myGPIOA_Init() {
	/* Enable clock for GPIOA peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	GPIO_InitTypeDef GPIO_InitStructInfo;
	GPIO_InitTypeDef* GPIO_InitStruct = &GPIO_InitStructInfo;
	GPIO_InitStruct->GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct->GPIO_Mode = 3;
	GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,GPIO_InitStruct);

	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
}

void myGPIOB_Init(){
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructInfo;
		GPIO_InitTypeDef* GPIO_InitStruct = &GPIO_InitStructInfo;
		GPIO_InitStruct->GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;
		GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, GPIO_InitStruct);
		GPIOB->AFR[0] &= ~(0xF0F000);

		GPIO_InitStruct->GPIO_Pin = LCD_LCK_PIN;
		GPIO_InitStruct->GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, GPIO_InitStruct);
}

void myTIM2_Init() {
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= 0x1;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 |= 0x84;

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	//TIM2->EGR[6] = 1; not in provided tim2 config
	TIM2->EGR |= 0x1;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	// NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC->IP[3] = ((uint32_t)0x00FFFFFF);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	//NVIC_EnableIRQ(TIM2_IRQn);
	NVIC->ISER[0] = ((uint32_t)0x00008000);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}

void myEXTI_Init() {
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= 0xff0f;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= 0x2;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= 0x2;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	return;
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler() {
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0) {
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= 0xFFFE;

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= 0x1;
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler() {
	uint32_t count = 0;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0) {
		if (edge_count < 1) {	// If the first edge, clear counter and start counting
			TIM2->CNT = 0;
			TIM2->CR1 |= 0x1;
			edge_count++;
		} else {
/* If the second edge, save the counter value and calculate the period and frequency of the clock cycle */
			TIM2->CR1 &= 0xFFFE;
			count = TIM2->CNT;
			period = ((double) count) / ((double) 48000000);
			freq = ((double) 1) / period;
			edge_count = 0;
		}
		EXTI->PR |= (EXTI_PR_PR1); // Clear the pending interrupt flag
	}
}

void myADC_Init(){
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	ADC1->CR = ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0);
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) != 1);
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;
	ADC1->CFGR1 |= ADC_CFGR1_CONT|ADC_CFGR1_OVRMOD;
	ADC1->CR |=0x4;
}

void myDAC_Init(){
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR |= DAC_CR_EN1;
}

void move_LCD_cursor(uint8_t address){
	Send_LCD(address + 0x80,CMD);
	myDelay(2);
}

void write_upper(){
	int frequency = freq;
	move_LCD_cursor(0x02);
	char str[5];

	snprintf(str,5,"%04d",frequency);
	for(int i = 0 ; i < 4; i ++){
		Send_LCD(str[i],DATA)
	}
}

void write_lower(){
	move_LCD_cursor(0x42);
	char str[5];
	snprintf(str,5,"%04d",res);
	for(int i = 0 ; i < 4; i ++){
		Send_LCD(str[i],DATA);
	}
}

void write_all(){
	write_upper();
	write_lower();
}

uint16_t get_res(){
	while((ADC1->ISR & ADC_ISR_EOSMP) == 0);
	uint16_t ADC_raw = ADC1->DR;
	double Vdd = ((double)ADC_raw)/((double)4095);
	res =round(Vdd*5000);
	return ADC_raw;
}

void Set_DAC(uint16_t DAC_Raw){
	uint16_t bottom = 1365;
	double DAC_norm = ((double)DAC_Raw)/((double)4095);
	DAC->DHR12R1 = (uint16_t)((4095 - 1365)*DAC_norm + bottom);
}


#pragma GCC diagnostic pop
