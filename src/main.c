/* Includes */
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"
#include "string.h"

#include "pwm.h"
#include "encoder.h"
#include "timer.h"
#include "PID_C3.h"

/* Private macro */
/* Private variables */
uint32_t start;
uint32_t stop;
uint32_t result;

float setpoint;
float input;
float output;

DCL_PID PID_MOTOR = PID_DEFAULTS;

/* Private function prototypes */
void LED_Init(void);
static void SystemClock_Config(void);
static inline void PID_POSITION(void);

/* Private functions */


int main(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->LAR = 0xC5ACCE55;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable DWT

	SCB_EnableDCache();
	SCB_EnableICache();

	HAL_Init();
	SystemClock_Config();

	LED_Init();
	PWM1_Init();
	ENCODER2_Init();
	ENCODER5_Init();
	TIMER7_Init(50);

	PID_MOTOR.Kp = 15.2f;
	PID_MOTOR.Ki = 2.5f;
	PID_MOTOR.Kd = 0.001f;
	PID_MOTOR.Umax = 1000.0f;
	PID_MOTOR.Umin = -1000.0f;



	while(1)
	{
//		GPIOB->ODR ^= (1<<7);
//		DWT->CYCCNT = 0;
//		start = DWT->CYCCNT;
//
//
//		stop = DWT->CYCCNT;
//		result = stop - start;
	}
}

void TIM7_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&TIM7_Handler,TIM_IT_UPDATE);
	PID_POSITION();
}

static inline void PID_POSITION(void)
{
	input = TIM2->CNT;
	setpoint = __HAL_TIM_GET_COUNTER(&TIM5_Handler);
	output = DCL_runPID_C3(&PID_MOTOR, setpoint, input, 0.001f);
	PWM_Control((int32_t)output);
}

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;


  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);


  /* Activate the OverDrive to reach the 216 MHz Frequency */
  HAL_PWREx_EnableOverDrive();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

}
