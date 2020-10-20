#include <string.h>
#include "stm32f1xx_hal.h"
#include "mgmt_hw.h"
#include "app.h"

#define ADC_CHS		(5)

/* buttons pins */
#define BTNX_GPIO_Port 	GPIOB
#define BTN1_Pin 	GPIO_PIN_0
#define BTN2_Pin 	GPIO_PIN_1
/* output pins */
#define LED_Pin 	GPIO_PIN_13
#define LED_GPIO_Port 	GPIOC
#define LDOUT_Pin 	GPIO_PIN_8
#define LDOUT_GPIO_Port GPIOA
#define RELAY_Pin 	GPIO_PIN_9
#define RELAY_GPIO_Port GPIOB
/* SPI pins*/

static volatile uint32_t ticks;
static uint8_t curr_adc;
static uint32_t adc_vals[ADC_CHS];

static const uint16_t counters[] = 
{
	160, 176, 193, 212, 232, 255, 280, 307, 338, 371, 407, 447, 
	490, 538, 591, 649, 712, 782, 858, 942, 1034, 1135, 1246, 
	1368, 1502, 1649, 1810, 1987, 2182, 2395, 2629, 2886, 3169, 
	3479, 3819, 4192, 4602, 5052, 5546, 6089, 6684, 7338, 8056, 
	8844, 9709, 10658, 11701, 12845, 14101, 15480, 16994, 
	18656, 20481, 22484, 24683, 27097, 29747, 32656, 35850, 
	39356, 43205, 47431, 52070, 57162, 
};

#define MAX_PWM		(sizeof(counters)/sizeof(counters[0]))

static uint8_t irq_cnt;
uint32_t pwms[LEDS_CNT];

static void Error_Handler(void)
{
	for (;;);
}

static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/* Initializes the CPU, AHB and APB busses clocks */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Initializes the CPU, AHB and APB busses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
		RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | 
		RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, 
				FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_SPI2_Init(void)
{
	/* enable clock */
	__HAL_RCC_SPI2_CLK_ENABLE();

	/* disable SPI2 */
	SPI2->CR1 &= SPI_CR1_SPE;

	/* set prescaler */
	SPI2->CR1 |= SPI_BAUDRATEPRESCALER_4;

	/* master mode */
	SPI2->CR1 |= SPI_MODE_MASTER;

	/* do not use slave select */
	SPI2->CR1 |= SPI_CR1_SSM;
	//SPI2->CR1 &= ~SPI_CR1_SSI;

	/* enable SPI2 */
	SPI2->CR1 |= SPI_CR1_SPE;
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LDOUT_GPIO_Port, LDOUT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RELAY_Pin */
	GPIO_InitStruct.Pin = RELAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LDOUT_Pin  */
	GPIO_InitStruct.Pin = LDOUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LDOUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN1_Pin BTN2_Pin */
	GPIO_InitStruct.Pin = BTN1_Pin | BTN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BTNX_GPIO_Port, &GPIO_InitStruct);
	
	/* configure ADC pins */
	memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
		GPIO_PIN_3 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIOA->CRL &= 0xFFFFFC00;
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* configure SPI pins */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_ADC1_Init(void)
{
	__HAL_RCC_ADC1_CLK_ENABLE();

	ADC1->CR2 |= ADC_CR2_ADON;
	while((ADC1->CR2 | ADC_CR2_ADON) == 0);
	ADC1->CR2 |= ADC_CR2_EXTSEL;

	ADC1->SMPR1 = 0x7FFFFFFF;
	ADC1->SMPR2 = 0xCFFFFFFF;
}

static void pwm_timer_init(void)
{
	/* enable clock */
	__HAL_RCC_TIM4_CLK_ENABLE();

	/* set prescaler */
	TIM4->PSC = 0;

	/* schedule first timer interrupt */
	TIM4->ARR = 0xFFFE;

	/* enable counter */
	TIM4->CR1 |= TIM_CR1_CEN;

	/* enable counter interrupt */
	TIM4->DIER |= TIM_DIER_UIE;

	/* enable TIM4 interrupt */
	NVIC_EnableIRQ(TIM4_IRQn);
}

void SysTick_Handler (void)
{
	HAL_IncTick();
}

void mgmt_hw_init(void)
{
	SystemClock_Config();
	//SystickInit(APP_TICK_FREQ);
	HAL_Init();
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_ADC1_Init();
	pwm_timer_init();
}

void mgmt_hw_fsm(void)
{
	static uint32_t old;
	uint32_t tick;

	tick = HAL_GetTick() / 50;

	if (tick != old)
	{
		old = tick;

		adc_vals[curr_adc] = (adc_vals[curr_adc] + ADC1->DR) / 2;

		if (++curr_adc >= ADC_CHS)
		{
			curr_adc = 0;
		}

		ADC1->SQR3 = (curr_adc << (0 * 5));
		ADC1->CR2 |= ADC_CR2_SWSTART;
		ADC1->CR2 &= ~ADC_CR2_SWSTART;
	}
}

/* 
 * macro to set the PWM PIN output
 * p is the PWM index
 * b is the byte in tx buffer (the shift register index)
 * i is the bit index of the byte
 */
#define SET_OUT(p, b, i)		\
	if (irq_cnt < pwms[(p)]) 		\
	 buff[(b)] |= (1 << (i)); else\
	 buff[(b)] &= ~(1 << (i))

void __attribute__ ((optimize ("O3"))) TIM4_IRQHandler(void)
//void TIM4_IRQHandler(void)
{
	uint8_t next;
	static uint8_t buff[3];

	/* clear the interrupt flag */
	TIM4->SR &= 0xFFFFFE;

	/* schedule the next timer interrupt asap */
	next = irq_cnt + 1;

	if (next >= MAX_PWM)
	{
		next = 0;
	}

	TIM4->ARR = counters[next];

	/* Set the LEDs output 
	 *
	 * This loop has been unrolled to be fast, it's CPU 
	 * time spent is very predictable.
	 * One can choose the shift register output PIN here,
	 * I have 17 pwm outputs, so I use 6 pins of the
	 * first shift register, 6 of the second and 5 of the
	 * third.
	 * I'm using the 74HC595, so I do not use the Qa pin
	 * because it's on the other side of the DIP package 
	 * and the routing will be complicated for nothing :p
	 */
	SET_OUT(0, 0, 3);
	SET_OUT(1, 0, 4);
	SET_OUT(2, 0, 5);
	SET_OUT(3, 0, 6);
	SET_OUT(4, 0, 7);

	SET_OUT(5, 1, 2);
	SET_OUT(6, 1, 3);
	SET_OUT(7, 1, 4);
	SET_OUT(8, 1, 5);
	SET_OUT(9, 1, 6);
	SET_OUT(10, 1, 7);

	SET_OUT(11, 2, 2);
	SET_OUT(12, 2, 3);
	SET_OUT(13, 2, 4);
	SET_OUT(14, 2, 5);
	SET_OUT(15, 2, 6);
	SET_OUT(16, 2, 7);

	/* increase irq counter */
	irq_cnt = next;

	/* send spi packet */
	SPI2->DR = buff[2];
	while((SPI2->SR & SPI_SR_TXE) == 0);
	SPI2->DR = buff[1];
	while((SPI2->SR & SPI_SR_TXE) == 0);
	SPI2->DR = buff[0];
	while((SPI2->SR & SPI_SR_BSY) != 0);

	/* generate clock pulse */
	HAL_GPIO_WritePin(LDOUT_GPIO_Port, LDOUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LDOUT_GPIO_Port, LDOUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LDOUT_GPIO_Port, LDOUT_Pin, GPIO_PIN_RESET);
}

uint8_t mgmt_hw_get_button(uint8_t id)
{
	uint8_t val;

	switch (id)
	{
	case 0:
		val = HAL_GPIO_ReadPin(BTNX_GPIO_Port, BTN1_Pin);
		break;
	case 1:
		val = HAL_GPIO_ReadPin(BTNX_GPIO_Port, BTN2_Pin);
		break;

	default:
		val = GPIO_PIN_SET;
	}

	return val == GPIO_PIN_RESET;
}

uint32_t mgmt_hw_get_app_tick(void)
{
	return HAL_GetTick() / 10;
}

void mgmt_hw_led_status(uint8_t state)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 
			state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

uint8_t mgmt_hw_get_adc(enum mgmt_hw_adc_e idx)
{
	uint32_t val;

	switch (idx)
	{
	case ADC_STAIR_TIME:
		val = adc_vals[0];
		break;

	case ADC_STEP_TIME:
		val = adc_vals[1];
		break;

	case ADC_RAMP_ON:
		val = adc_vals[2];
		break;

	case ADC_RAMP_OFF:
		val = adc_vals[3];
		break;

	case ADC_PWM_MAX:
		val = adc_vals[4];
		break;

	default:
		val = 0;
		break;
	}

	return val >> 4;
}

void mgmt_hw_set_relay(uint8_t state)
{
	HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t mgmt_hw_get_max_pwm(void)
{
	return MAX_PWM + 1;
}
