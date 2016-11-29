//====================================================================
// INCLUDE FILES
//====================================================================
#include <stdint.h>
#include "LCD.h"
#include "stm32f0xx.h"
#include <stm32f0xx_adc.h>
#include <stdio.h>
//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//Lift Motor 	- PA1 	Timer 2		CCR2
//Left Motor 	- PB11 	Timer 2		CCR3
//Right Motor 	- PB10 	Timer 2		CCR4
//Center Motor 	- PB1	Timer 3		CCR4
//Ultrasonic 	- PB0 	Timer 3 	CCR3

//====================================================================
// GLOBAL VARIABLES
//====================================================================

//Input Capture

uint16_t IC3ReadValue1 = 0, IC3ReadValue2 = 0;
uint16_t CaptureNumber = 0;
uint32_t Capture = 0;
uint32_t distanceReading = 0;

uint32_t prevDistance = 0;
uint32_t speed = 0;
char conversionBuffer[8];

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================

void initButtons(void);
void initADC(void);
void initTimer1(void);
void initTimer2(void);
void initTimer3(void);

void delay(uint32_t value);
void displayLCD(void);
uint32_t getSpeed(void);
char* intToString(uint32_t number);

void TIM1_CC_IRQHandler(void);


//====================================================================
// MAIN FUNCTION
//====================================================================

int main(void)
{
	//initialization Functions
	initButtons();
	lcd_init();
	initTimer1();
	initTimer2();
	initTimer3();

	while(1)
	{
		displayLCD();
		prevDistance = distanceReading;
		delay(500);
	}
	return 0;
}

//====================================================================
// INITIALISE FUNCTIONS
//====================================================================

void initTimer1(void)
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//TIM1 clock enable 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	//GPIOA clock enable
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//TIM1 channel 2 pin (PE.11) configuration 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Connect TIM pins to AF2 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);

	//TIM1 configuration: Input Capture mode ---------------------
	//   The external signal is connected to TIM1 CH2 pin (PA.09)
	//   The Rising edge is used as active edge,
	//  The TIM1 CCR2 is used to compute the frequency value
	//------------------------------------------------------------ 

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM1->PSC |= 32;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//TIM enable counter 
	TIM_Cmd(TIM1, ENABLE);

	//Enable the CC2 Interrupt Request 
	TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);

	//Enable the TIM1 global Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void initTimer2(void) 
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	GPIOA->MODER |= GPIO_MODER_MODER1_1;
	GPIOB->MODER |= GPIO_MODER_MODER10_1; // PB10 = AF
	GPIOB->MODER |= GPIO_MODER_MODER11_1; // PB11 = AF

	GPIOA->AFR[0] |= (2 << 4);
	GPIOB->AFR[1] |= (2 << (4*(10 - 8))); // PB10_AF = AF2 (ie: map to TIM2_CH3)
	GPIOB->AFR[1] |= (2 << (4*(11 - 8))); // PB11_AF = AF2 (ie: map to TIM2_CH4)

	TIM2->ARR = 960000;  // f = 50Hz
	// specify PWM mode: OCxM bits in CCMRx. We want mode 1
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); // PWM Mode 1
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1

	// set PWM percantages
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;

	// enable the OC channels
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->CCER |= TIM_CCER_CC4E;
	TIM2->CCER |= TIM_CCER_CC2E;

	TIM2->CR1 |= TIM_CR1_CEN; // counter enable
}

void initTimer3(void) 
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	GPIOB->MODER |= GPIO_MODER_MODER0_1;
	GPIOB->MODER |= GPIO_MODER_MODER1_1;

	GPIOB->AFR[0] |= (1 << 0);
	GPIOB->AFR[0] |= (1 << 4);

	TIM3->PSC |= 5000;
	TIM3->ARR = 960;		// f = 10 Hz

	// specify PWM mode: OCxM bits in CCMRx. We want mode 1
	TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
	TIM3->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);

	// set PWM percentages
	TIM3->CCR3 = 1; // 104us - PB0 for ultra-sonic sensor
	TIM3->CCR4 = 0; // center motor

	// enable the OC channels
	TIM3->CCER |= TIM_CCER_CC3E;
	TIM3->CCER |= TIM_CCER_CC4E;

	TIM3->CR1 |= TIM_CR1_CEN;
}

void initButtons(void) 
{	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable clock for push-buttons
	// set pins to inputs
	GPIOA->MODER &= ~GPIO_MODER_MODER0; //set PA0 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER1; //set PA2 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER2; //set PA3 to input
	GPIOA->MODER &= ~GPIO_MODER_MODER3; //set PA3 to input
	// enable pull-up resistors
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0; //enable pull up for PA0
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0; //enable pull up for PA1
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0; //enable pull up for PA2
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0; //enable pull up for PA3
}

void initADC(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //enable clock for ADC					
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable clock for port
	GPIOA->MODER |= GPIO_MODER_MODER6; //set PA6 to analog mode
	ADC1->CHSELR |= ADC_CHSELR_CHSEL6;// select channel 6
	ADC1->CFGR1 |= ADC_CFGR1_RES_1; // resolution to 8 bit
	ADC1->CR |= ADC_CR_ADEN; // set ADEN=1 in the ADC_CR register
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0); //wait until ADRDY==1 in ADC_ISR
}

//====================================================================
// FUNCTIONS
//====================================================================

void delay(uint32_t value)
{
    volatile uint32_t i, k;

	for (i = 0; i < value; i++)
		{for (k = 0; k < value; k++);}
}

void displayLCD(void)
{
	lcd_command(LCD_CLEAR_DISPLAY);
	lcd_string("Distance: ");
	//lcd_string(intToString(distanceReading));
	lcd_string(" cm");
	lcd_command(LCD_GOTO_LINE_2);
	lcd_string("Speed: ");
	//lcd_string(intToString(getSpeed()));

	char* string_to_print = "Hello";
  	while (string_to_print[count] != 0) 
	{
    	lcd_put (string_to_print[count], TEXT);
    	count++;
	}

}

uint32_t getSpeed(void)
{
	return prevDistance - distanceReading;
}

char* intToString(uint32_t number)
{
	sprintf(conversionBuffer, "%d", number);
	return conversionBuffer;
}

//====================================================================
// INTERRUPTS
//====================================================================


void TIM1_CC_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET)
  {
    //Clear TIM1 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    if(CaptureNumber == 0)
    {
      //Get the Input Capture value 
      IC3ReadValue1 = TIM_GetCapture2(TIM1);
      CaptureNumber = 1;
    }
    else if(CaptureNumber == 1)
    {
      //Get the Input Capture value 
      IC3ReadValue2 = TIM_GetCapture2(TIM1);

      //Capture computation 
      if (IC3ReadValue2 > IC3ReadValue1)
      {
        Capture = (IC3ReadValue2 - IC3ReadValue1);
      }
      else if (IC3ReadValue2 < IC3ReadValue1)
      {
        Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2);
      }
      else
      {
        Capture = 0;
      }
      //Frequency computation 
      distanceReading = (uint32_t)(((Capture*32)/48)/58);
      CaptureNumber = 0;
    }
  }
}

