/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "stdio.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId ENC1TaskHandle;
osThreadId ENC2TaskHandle;
osThreadId ENC3TaskHandle;
osThreadId ENC4TaskHandle;
/* USER CODE BEGIN PV */
osThreadId motorTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void const * argument);
void StartENC1Task(void const * argument);
void StartENC2Task(void const * argument);
void StartENC3Task(void const * argument);
void StartENC4Task(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin );
void StartMotorTask(void const * argument);
void ComputeMotorPID(float *motor_1_ang_vel, float *motor_2_ang_vel, float *motor_3_ang_vel, float *motor_4_ang_vel);

//Callback declarations
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool enable_motors = false;

//Encoder 1 variables
uint8_t icflag1_1 = 0;
uint8_t icflag2_1 = 0;
uint32_t tick1_1;
uint32_t tick2_1;
uint32_t period1;
float frequency1 = 0.0;
float motor1_ang_vel = 0.0;
uint32_t start_tick1;

//Encoder 2 variables
uint8_t icflag1_2 = 0;
uint8_t icflag2_2 = 0;
uint32_t tick1_2;
uint32_t tick2_2;
uint32_t period2;
float frequency2 = 0.0;
float motor2_ang_vel = 0.0;
uint32_t start_tick2;

//Encoder 3 variables
uint8_t icflag1_3 = 0;
uint8_t icflag2_3 = 0;
uint32_t tick1_3;
uint32_t tick2_3;
uint32_t period3;
float frequency3 = 0.0;
float motor3_ang_vel = 0.0;
uint32_t start_tick3;

//Encoder 4 variables
uint8_t icflag1_4 = 0;
uint8_t icflag2_4 = 0;
uint32_t tick1_4;
uint32_t tick2_4;
uint32_t period4;
float frequency4 = 0.0;
float motor4_ang_vel = 0.0;
uint32_t start_tick4;

//Wheel ang vel PID variables
uint32_t motor_current_time = 0;
uint32_t motor_previous_time = 0;
float motor_elapsed_time;

float motor1_error;
float motor1_cum_error = 0;
float motor1_cum_error_previous = 0;
float motor1_rate_error = 0;
float motor1_last_error = 0;
float motor1_setpoint = 20.0;
float motor1_output_voltage;
float motor1_max_voltage = 4.5;
uint32_t motor1_PWM;

float motor2_error;
float motor2_cum_error = 0;
float motor2_cum_error_previous = 0;
float motor2_rate_error = 0;
float motor2_last_error = 0;
float motor2_setpoint = 20.0;
float motor2_output_voltage;
float motor2_max_voltage = 4.5;
uint32_t motor2_PWM;

float motor3_error;
float motor3_cum_error = 0;
float motor3_cum_error_previous = 0;
float motor3_rate_error = 0;
float motor3_last_error = 0;
float motor3_setpoint = 20.0;
float motor3_output_voltage;
float motor3_max_voltage = 4.5;
uint32_t motor3_PWM;

float motor4_error;
float motor4_cum_error = 0;
float motor4_cum_error_previous = 0;
float motor4_rate_error = 0;
float motor4_last_error = 0;
float motor4_setpoint = 20.0;
float motor4_output_voltage;
float motor4_max_voltage = 4.5;
uint32_t motor4_PWM;

float Kp = 0.02;
float Ki = 0.00015;
float Kd = 0;



//printf function
int _write(int file, char *ptr, int len)
{
	int i=0;
	for(i=0; i<len; i++) {
		ITM_SendChar((*ptr++));
	}
	return len;
}

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // Start PWM for motor drivers
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  //HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  //HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  //HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ENC1Task */
  osThreadDef(ENC1Task, StartENC1Task, osPriorityNormal, 0, 128);
  ENC1TaskHandle = osThreadCreate(osThread(ENC1Task), NULL);

  /* definition and creation of ENC2Task */
  osThreadDef(ENC2Task, StartENC2Task, osPriorityNormal, 0, 128);
  ENC2TaskHandle = osThreadCreate(osThread(ENC2Task), NULL);

  /* definition and creation of ENC3Task */
  osThreadDef(ENC3Task, StartENC3Task, osPriorityNormal, 0, 128);
  ENC3TaskHandle = osThreadCreate(osThread(ENC3Task), NULL);

  /* definition and creation of ENC4Task */
  osThreadDef(ENC4Task, StartENC4Task, osPriorityNormal, 0, 128);
  ENC4TaskHandle = osThreadCreate(osThread(ENC4Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(motorTask, StartMotorTask, osPriorityNormal, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1292;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65536 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65536 - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|TRIG_2_Pin|TRIG_4_Pin|BIN1_1_Pin
                          |STBY_1_Pin|AIN1_1_Pin|AIN2_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_5_Pin|LD2_Pin|TRIG_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIG_1_Pin|BIN2_2_Pin|BIN1_2_Pin|STBY2_Pin
                          |AIN1_2_Pin|AIN2_2_Pin|BIN2_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 TRIG_2_Pin TRIG_4_Pin BIN1_1_Pin
                           STBY_1_Pin AIN1_1_Pin AIN2_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|TRIG_2_Pin|TRIG_4_Pin|BIN1_1_Pin
                          |STBY_1_Pin|AIN1_1_Pin|AIN2_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_5_Pin LD2_Pin TRIG_3_Pin */
  GPIO_InitStruct.Pin = TRIG_5_Pin|LD2_Pin|TRIG_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_1_Pin BIN2_2_Pin BIN1_2_Pin STBY2_Pin
                           AIN1_2_Pin AIN2_2_Pin BIN2_1_Pin */
  GPIO_InitStruct.Pin = TRIG_1_Pin|BIN2_2_Pin|BIN1_2_Pin|STBY2_Pin
                          |AIN1_2_Pin|AIN2_2_Pin|BIN2_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void StartMotorTask(void const * argument)
{
  for(;;)
  {
		//Start motors drivers for 1 and 2
		HAL_GPIO_WritePin(AIN2_1_GPIO_Port, AIN2_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN1_1_GPIO_Port, AIN1_1_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(STBY_1_GPIO_Port, STBY_1_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(BIN2_1_GPIO_Port, BIN2_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN1_1_GPIO_Port, BIN1_1_Pin, GPIO_PIN_RESET);

		//Start motors drivers for 3 and 4
		HAL_GPIO_WritePin(AIN2_2_GPIO_Port, AIN2_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN1_2_GPIO_Port, AIN1_2_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(STBY2_GPIO_Port, STBY2_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(BIN2_2_GPIO_Port, BIN2_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN1_2_GPIO_Port, BIN1_2_Pin, GPIO_PIN_SET);

		//Compute PID values for low lever angular velocity controller
		ComputeMotorPID(&motor1_ang_vel, &motor2_ang_vel, &motor3_ang_vel, &motor4_ang_vel);
		motor1_PWM = (int)(255*fabs(motor1_output_voltage)/motor1_max_voltage);
		motor2_PWM = (int)(255*fabs(motor2_output_voltage)/motor2_max_voltage);
		motor3_PWM = (int)(255*fabs(motor3_output_voltage)/motor3_max_voltage);
		motor4_PWM = (int)(255*fabs(motor4_output_voltage)/motor4_max_voltage);

		if (motor1_PWM > 255) {
			motor1_PWM = 255;
		}
		if (motor2_PWM > 255) {
			motor2_PWM = 255;
		}
		if (motor3_PWM > 255) {
			motor3_PWM = 255;
		}
		if (motor4_PWM > 255) {
			motor4_PWM = 255;
		}

		if(enable_motors){

		  //Set PWM values for 1 and 2
		  htim1.Instance->CCR4 = motor1_PWM;
		  htim1.Instance->CCR3 = motor2_PWM;

		  //Set PWM values for 3 and 4
		  htim1.Instance->CCR2 = motor3_PWM;
		  htim1.Instance->CCR1 = motor4_PWM;

		}
		else{

		  //Set PWM values for 1 and 2
		  htim1.Instance->CCR4 = 0;
		  htim1.Instance->CCR3 = 0;

		  //Set PWM values for 3 and 4
		  htim1.Instance->CCR2 = 0;
		  htim1.Instance->CCR1 = 0;

		}



	  osDelay(100);
  }
}

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin )
{
  if (GPIO_Pin == B1_Pin) {
	  enable_motors = !enable_motors;
	  HAL_GPIO_TogglePin (LD2_GPIO_Port, LD2_Pin);
  }
  else {
	  __NOP ();
  }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (icflag1_1 == 0){
			tick1_1 = __HAL_TIM_GetCounter(htim);
			icflag1_1 = 1;
		}
		else if (icflag1_1 == 1){
			tick2_1 = __HAL_TIM_GetCounter(htim);
			icflag2_1 = 1;
		}
	}

	else if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (icflag1_2 == 0){
			tick1_2 = __HAL_TIM_GetCounter(htim);
			icflag1_2 = 1;
		}
		else if (icflag1_2 == 1){
			tick2_2 = __HAL_TIM_GetCounter(htim);
			icflag2_2 = 1;
		}
	}
	else if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if (icflag1_3 == 0){
			tick1_3 = __HAL_TIM_GetCounter(htim);
			icflag1_3 = 1;
		}
		else if (icflag1_3 == 1){
			tick2_3 = __HAL_TIM_GetCounter(htim);
			icflag2_3 = 1;
		}
	}
	else if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if (icflag1_4 == 0){
			tick1_4 = __HAL_TIM_GetCounter(htim);
			icflag1_4 = 1;
		}
		else if (icflag1_4 == 1){
			tick2_4 = __HAL_TIM_GetCounter(htim);
			icflag2_4 = 1;
		}
	}
}

void ComputeMotorPID(float *motor_1_ang_vel, float *motor_2_ang_vel, float *motor_3_ang_vel, float *motor_4_ang_vel)
{

	motor_current_time = HAL_GetTick();
	motor_elapsed_time = (float)(motor_current_time - motor_previous_time);

	//Calculate the proportional term
	motor1_error = motor1_setpoint - *motor_1_ang_vel;
	motor2_error = motor2_setpoint - *motor_2_ang_vel;
	motor3_error = motor3_setpoint - *motor_3_ang_vel;
	motor4_error = motor4_setpoint - *motor_4_ang_vel;

	//Calculate the integral term
	motor1_cum_error = motor1_cum_error_previous + motor1_error*motor_elapsed_time;
	motor2_cum_error = motor2_cum_error_previous + motor2_error*motor_elapsed_time;
	motor3_cum_error = motor3_cum_error_previous + motor3_error*motor_elapsed_time;
	motor4_cum_error = motor4_cum_error_previous + motor4_error*motor_elapsed_time;

	//Calculate the derivative term
	motor1_rate_error = (motor1_error - motor1_last_error)/motor_elapsed_time;
	motor2_rate_error = (motor2_error - motor2_last_error)/motor_elapsed_time;
	motor3_rate_error = (motor3_error - motor3_last_error)/motor_elapsed_time;
	motor4_rate_error = (motor4_error - motor4_last_error)/motor_elapsed_time;

	//Main voltage input to the motors
	motor1_output_voltage = Kp*motor1_error + Ki*motor1_cum_error + Kd*motor1_rate_error;
	motor2_output_voltage = Kp*motor2_error + Ki*motor2_cum_error + Kd*motor2_rate_error;
	motor3_output_voltage = Kp*motor3_error + Ki*motor3_cum_error + Kd*motor3_rate_error;
	motor4_output_voltage = Kp*motor4_error + Ki*motor4_cum_error + Kd*motor4_rate_error;

	//Make sure the input voltage does not exceed max value
	if (motor1_output_voltage > motor1_max_voltage){
		motor1_output_voltage = motor1_max_voltage;
		motor1_cum_error = motor1_cum_error_previous;
	}

	if (motor2_output_voltage > motor2_max_voltage){
		motor2_output_voltage = motor2_max_voltage;
		motor2_cum_error = motor2_cum_error_previous;
	}

	if (motor3_output_voltage > motor3_max_voltage){
		motor3_output_voltage = motor3_max_voltage;
		motor3_cum_error = motor3_cum_error_previous;
	}

	if (motor4_output_voltage > motor4_max_voltage){
		motor4_output_voltage = motor4_max_voltage;
		motor4_cum_error = motor4_cum_error_previous;
	}

	//Loop resets
	motor1_last_error = motor1_error;
	motor1_cum_error_previous = motor1_cum_error;

	motor2_last_error = motor2_error;
	motor2_cum_error_previous = motor2_cum_error;

	motor3_last_error = motor3_error;
	motor3_cum_error_previous = motor3_cum_error;

	motor4_last_error = motor4_error;
	motor4_cum_error_previous = motor4_cum_error;

	motor_previous_time = motor_current_time;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartENC1Task */
/**
* @brief Function implementing the ENC1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartENC1Task */
void StartENC1Task(void const * argument)
{
  /* USER CODE BEGIN StartENC1Task */
  /* Infinite loop */
  for(;;)
  {
	//Start the timer in input capture mode
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);

	//get the first tick of the timer
	start_tick1 = HAL_GetTick();

	//Reset the timer
	__HAL_TIM_SetCounter(&htim5, 0);

	//Run the do while loop to get the elapsed period
	do
	{
		if(icflag2_1) break;
	}while (HAL_GetTick() - start_tick1 < 800);
	icflag1_1 = 0;
	icflag2_1 = 0;

	//Stop the timer
	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);

	//Calculate the elapsed period
	if (tick2_1 > tick1_1) {
		period1 = tick2_1 - tick1_1;
	}
	else {
		period1 = period1;
	}

	//Calculate the frequency and angular velocity
	if (period1 > 0 && period1 < 700){
		frequency1 = (float)10000/(period1);
		motor1_ang_vel = frequency1*(M_PI/10);
	}


	//Print the elapsed period
	printf("Angular Velocity 1: %d \n", (int)motor1_ang_vel);

    osDelay(200);
  }
  /* USER CODE END StartENC1Task */
}

/* USER CODE BEGIN Header_StartENC2Task */
/**
* @brief Function implementing the ENC2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartENC2Task */
void StartENC2Task(void const * argument)
{
  /* USER CODE BEGIN StartENC2Task */
  /* Infinite loop */
  for(;;)
  {
	  //Start the timer in input capture mode
	  	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	  	//get the first tick of the timer
	  	start_tick2 = HAL_GetTick();

	  	//Reset the timer
	  	__HAL_TIM_SetCounter(&htim4, 0);

	  	//Run the do while loop to get the elapsed period
	  	do
	  	{
	  		if(icflag2_2) break;
	  	}while (HAL_GetTick() - start_tick2 < 800);
	  	icflag1_2 = 0;
	  	icflag2_2 = 0;

	  	//Stop the timer
	  	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);

	  	//Calculate the elapsed period
	  	if (tick2_2 > tick1_2) {
	  		period2 = tick2_2 - tick1_2;
	  	}
	  	else {
	  		period2 = period2;
	  	}

	  	//Calculate the frequency and angular velocity
	  	if (period2 > 0 && period2 < 700){
	  		frequency2 = (float)10000/(period2);
	  		motor2_ang_vel = frequency2*(M_PI/10);
	  	}

	  	//Print the elapsed period
	  	printf("Angular Velocity 2: %d \n", (int)motor2_ang_vel);

	    osDelay(200);
  }
  /* USER CODE END StartENC2Task */
}

/* USER CODE BEGIN Header_StartENC3Task */
/**
* @brief Function implementing the ENC3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartENC3Task */
void StartENC3Task(void const * argument)
{
  /* USER CODE BEGIN StartENC3Task */
  /* Infinite loop */
  for(;;)
  {
	  //Start the timer in input capture mode
		HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

		//get the first tick of the timer
		start_tick3 = HAL_GetTick();

		//Reset the timer
		__HAL_TIM_SetCounter(&htim4, 0);

		//Run the do while loop to get the elapsed period
		do
		{
			if(icflag2_3) break;
		}while (HAL_GetTick() - start_tick3 < 800);
		icflag1_3 = 0;
		icflag2_3 = 0;

		//Stop the timer
		HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_2);

		//Calculate the elapsed period
		if (tick2_3 > tick1_3) {
			period3 = tick2_3 - tick1_3;
		}
		else {
			period3 = period3;
		}

		//Calculate the frequency and angular velocity
		if (period3 > 0 && period3 < 700){
			frequency3 = (float)10000/(period3);
			motor3_ang_vel = frequency3*(M_PI/10);
		}

		//Print the elapsed period
		printf("Angular Velocity 3: %d \n", (int)motor3_ang_vel);

		osDelay(200);
  }
  /* USER CODE END StartENC3Task */
}

/* USER CODE BEGIN Header_StartENC4Task */
/**
* @brief Function implementing the ENC4Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartENC4Task */
void StartENC4Task(void const * argument)
{
  /* USER CODE BEGIN StartENC4Task */
  /* Infinite loop */
  for(;;)
  {
	  //Start the timer in input capture mode
	  	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);

	  	//get the first tick of the timer
	  	start_tick4 = HAL_GetTick();

	  	//Reset the timer
	  	__HAL_TIM_SetCounter(&htim5, 0);

	  	//Run the do while loop to get the elapsed period
	  	do
	  	{
	  		if(icflag2_4) break;
	  	}while (HAL_GetTick() - start_tick4 < 800);
	  	icflag1_4 = 0;
	  	icflag2_4 = 0;

	  	//Stop the timer
	  	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2);

	  	//Calculate the elapsed period
	  	if (tick2_4 > tick1_4) {
	  		period4 = tick2_4 - tick1_4;
	  	}
	  	else {
	  		period4 = period4;
	  	}

	  	//Calculate the frequency and angular velocity
	  	if (period4 > 0 && period4 < 700){
	  		frequency4 = (float)10000/(period4);
	  		motor4_ang_vel = frequency4*(M_PI/10);
	  	}

	  	//Print the elapsed period
	  	printf("Angular Velocity 4: %d \n", (int)motor4_ang_vel);

	    osDelay(200);
  }
  /* USER CODE END StartENC4Task */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
