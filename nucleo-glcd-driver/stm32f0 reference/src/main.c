/*
 * Demo of glcd library with STM32F0 Discovery PCB and PCD8544 (Nokia 5110) LCD
 * - Andy Gock <andy@agock.com>
 *
 */

#include "main.h"

/* Function prototypes */
void delay_ms(uint32_t ms);
void setup_stm32f0discovery(void);

/* Global variables */
volatile uint32_t ticks;
extern volatile uint8_t unit_test_return; /* Signals demo function to return */

/* LEDs */
#define DISCOVERY_LD3_PORT GPIOC
#define DISCOVERY_LD3_PIN  GPIO_Pin_9
#define DISCOVERY_LD9_PORT GPIOC
#define DISCOVERY_LD9_PIN  GPIO_Pin_8

/* Push button 'user' */
#define DISCOVERY_B1_PORT GPIOA
#define DISCOVERY_B1_PIN  GPIO_Pin_0

/* Turn LEDs on or off */
#define DISCOVERY_LED3_ON()  GPIO_SetBits(GPIOC,GPIO_Pin_8);
#define DISCOVERY_LED3_OFF() GPIO_ResetBits(GPIOC,GPIO_Pin_8);
#define DISCOVERY_LED4_ON()  GPIO_SetBits(GPIOC,GPIO_Pin_9);
#define DISCOVERY_LED4_OFF() GPIO_ResetBits(GPIOC,GPIO_Pin_9);

int main(void)
{
	/* Set up systick timer to 1 millisecond */
	SysTick_Config(SystemCoreClock/1000);
	
	/* Initial set up of peripherals specific to the Discovery board */
	setup_stm32f0discovery();
	
	/* Initialise LCD */
	glcd_init();
	
	while(1) {
		/* Demo - switch between demo routines by pressing user push button */
		glcd_test_circles();
		glcd_test_counter_and_graph();
		glcd_test_text_up_down();
		glcd_test_tiny_text();
		glcd_test_hello_world();
		glcd_test_rectangles();
		glcd_test_scrolling_graph();	
	}
}

/* Set up STM32 Discovery board specific items using Standard Peripherals library */
void setup_stm32f0discovery(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set user button on STM32F0DISCOVERY (PA0) */
	GPIO_StructInit(&GPIO_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_PuPd  =  GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	/* Set up push button as interrupt */

	/* Connect EXTI0 Line to PA0 pin */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* Configure EXTI0 line - trigger both rising and falling edges */
	EXTI_InitStructure.EXTI_Line    = EXTI_Line0; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt; 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel         = EXTI0_1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Set GPIO for LD3, LD9 LEDs (PC9, PC8 respectively) */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
}

/* Discovery board button press and release interrupt */
void EXTI0_1_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line0) == SET)
  {
		/* STM32 Discovery button B1 was pressed or released */
		if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)) {
			/* Button is down - turn LED on */
			DISCOVERY_LED3_ON();
		} else {
			/* Button is up - turn LED off */
			DISCOVERY_LED3_OFF();
			unit_test_return = 1; /* Signal to demo function to return */
		}
		
		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

/* Systick interrupt runs every millisecond */
void SysTick_Handler(void)
{
	ticks++;
}

/* Delay by prescribed time in milliseconds */
void delay_ms(uint32_t ms) {
    uint32_t now = ticks;
    while ((ticks-now) < ms);
}

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval int8_t never returns
  */
int8_t assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Assert failed: file %s on line %d\r\n", file, line);
  
	/* Infinite loop */
  while (1)
  {
  }
}
