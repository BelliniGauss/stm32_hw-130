/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "hw_130.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//	Option to inverr default rotation direction for motors:
#define MOTOR_1_INVERT false
#define MOTOR_2_INVERT false
#define MOTOR_3_INVERT false
#define MOTOR_4_INVERT false


//	Mapping of Shift Register output to Motor Driver input.
#define MOTOR_SR_1_P 	2
#define MOTOR_SR_1_N	3
#define MOTOR_SR_2_P 	4
#define MOTOR_SR_2_N	1
#define MOTOR_SR_3_P 	7
#define MOTOR_SR_3_N	5
#define MOTOR_SR_4_P 	0
#define MOTOR_SR_4_N	6



TIM_HandleTypeDef htim2;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */







//	motor_pwm *pwm_interface,

//static void setDutyCycle(TIM_HandleTypeDef* const htim, uint32_t channel, float duty_cycle) {


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  hw_130_driver motor_driver =  create_hw_130(	SR_DATA_Pin, SR_DATA_GPIO_Port,
												SR_CLOCK_Pin, SR_CLOCK_GPIO_Port,
												SR_LATCH_Pin, SR_LATCH_GPIO_Port);




  motor_initialize(motor_driver,	0, &htim2, TIM_CHANNEL_1, MOTOR_SR_1_P, MOTOR_SR_1_N, MOTOR_1_INVERT);
  motor_initialize(motor_driver,	1, &htim2, TIM_CHANNEL_2, MOTOR_SR_2_P, MOTOR_SR_2_N, MOTOR_2_INVERT);
  motor_initialize(motor_driver, 	2, &htim2, TIM_CHANNEL_3, MOTOR_SR_3_P, MOTOR_SR_3_N, MOTOR_3_INVERT);
  motor_initialize(motor_driver,	3, &htim2, TIM_CHANNEL_4, MOTOR_SR_4_P, MOTOR_SR_4_N, MOTOR_4_INVERT);

  start_hw_130(	motor_driver);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  GPIOC->ODR &= ~(1<<13);		//	LED on
	  /*
	   * 4 motor change + update
	   * 156 uS @ 16 MHz -> ~2500 clock cycles.
	   */
	  int pwm = 75;
	  motor_rotation direction = forwards;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);


	  HAL_Delay(10000);

	  GPIOC->ODR = 1<<13;    		// led off

	  direction = stop;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);

	  HAL_Delay(3000);


	  //########################################àààà


	  GPIOC->ODR &= ~(1<<13);		//	LED on
	  	  /*
	  	   * 4 motor change + update
	  	   * 156 uS @ 16 MHz -> ~2500 clock cycles.
	  	   */
	  pwm = 85;
	  direction = forwards;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);


	  HAL_Delay(10000);

	  GPIOC->ODR = 1<<13;    		// led off

	  direction = stop;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);

	  HAL_Delay(3000);


	  //########################################àààà


	  GPIOC->ODR &= ~(1<<13);		//	LED on
		  /*
		   * 4 motor change + update
		   * 156 uS @ 16 MHz -> ~2500 clock cycles.
		   */
	  pwm = 95;
	  direction = forwards;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);


	  HAL_Delay(10000);

	  GPIOC->ODR = 1<<13;    		// led off

	  direction = stop;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);

	  HAL_Delay(3000);



	  //########################################àààà


	  GPIOC->ODR &= ~(1<<13);		//	LED on
		  /*
		   * 4 motor change + update
		   * 156 uS @ 16 MHz -> ~2500 clock cycles.
		   */
	  pwm = 100;
	  direction = forwards;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);


	  HAL_Delay(10000);

	  GPIOC->ODR = 1<<13;    		// led off

	  direction = stop;
	  motor_set(motor_driver, 0, direction, pwm);
	  motor_set(motor_driver, 1, direction, pwm);
	  motor_set(motor_driver, 2, direction, pwm);
	  motor_set(motor_driver, 3, direction, pwm);
	  update_motors(motor_driver);

	  HAL_Delay(3000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 512;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_GPIO_Port, LED_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SR_DATA_Pin|SR_CLOCK_Pin|SR_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GPIO_Pin */
  GPIO_InitStruct.Pin = LED_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SR_DATA_Pin SR_CLOCK_Pin SR_LATCH_Pin */
  GPIO_InitStruct.Pin = SR_DATA_Pin|SR_CLOCK_Pin|SR_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
