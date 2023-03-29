/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @code copyrights: Dhinesh Kumar S B.E EIE, M.Tech EDT (Embedded Domain)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define pins for rotary encoder
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOA
#define ANGLE_PER_PULSE 0.9 // degrees per encoder pulse
#define PULSES_PER_REV 400 // pulses per revolution
/* Define the pins for the relays */
#define RELAY1_Pin GPIO_PIN_8
#define RELAY1_GPIO_Port GPIOA
#define RELAY2_Pin GPIO_PIN_9
#define RELAY2_GPIO_Port GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;

osThreadId RelayControlHandle;
osThreadId EncoderContolHandle;
osThreadId PIDContolHandle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId myBinarySem02Handle;
osSemaphoreId myBinarySem03Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_FDCAN1_Init(void);
void RelayControlTask(void const * argument);
void EncoderTask(void const * argument);
void PIDControllerTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* definition and creation of myBinarySem02 */
  osSemaphoreDef(myBinarySem02);
  myBinarySem02Handle = osSemaphoreCreate(osSemaphore(myBinarySem02), 1);

  /* definition and creation of myBinarySem03 */
  osSemaphoreDef(myBinarySem03);
  myBinarySem03Handle = osSemaphoreCreate(osSemaphore(myBinarySem03), 1);

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
  /* definition and creation of RelayControl */
  osThreadDef(RelayControl, RelayControlTask, osPriorityNormal, 0, 128);
  RelayControlHandle = osThreadCreate(osThread(RelayControl), NULL);

  /* definition and creation of EncoderContol */
  osThreadDef(EncoderContol, EncoderTask, osPriorityBelowNormal, 0, 128);
  EncoderContolHandle = osThreadCreate(osThread(EncoderContol), NULL);

  /* definition and creation of PIDContol */
  osThreadDef(PIDContol, PIDControllerTask, osPriorityAboveNormal, 0, 128);
  PIDContolHandle = osThreadCreate(osThread(PIDContol), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
volatile int32_t counter = 0;
volatile int32_t temp = 0;
volatile int32_t buf;
volatile int32_t buffer, buff;
volatile float angle = 0.0;

/* PID parameters */
float Kp = 1;
float Ki = 0.2;
float Kd = 0.1;

/* Feedback parameters */
//volatile long counter = 0;
float setpoint = 45; // Setpoint in degrees
float feedback = 0;
float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;
float output = 0;
volatile float last_feedback;

/* Anti-windup parameters */
float windup_guard = 100;
float proportional_on_measurement = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{//semtake
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//xSemaphoreTakeFromISR(myBinarySem01Handle, portMAX_DELAY);
	if(GPIO_Pin == ENCODER_A_Pin) // Encoder pin A
		  {
		    if(HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin) == GPIO_PIN_RESET) // Check pin B to determine direction
		    {
		      counter++; // Increment counter
		    }
		    else
		    {
		      counter--; // Decrement counter
		    }
		  }

		  else if(GPIO_Pin == ENCODER_B_Pin) // Encoder pin B
		  {
		    if(HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin) == GPIO_PIN_RESET) // Check pin A to determine direction
		    {
		      counter--; // Decrement counter
		    }
		    else
		    {
		      counter++; // Increment counter
		    }
		  }

	xTaskNotifyFromISR( EncoderContolHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
	  xSemaphoreGiveFromISR(myBinarySem01Handle, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	//semgive
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_RelayControlTask */
/**
  * @brief  Function implementing the RelayControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RelayControlTask */
void RelayControlTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	 // HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
	  /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET);
    osDelay(1000);*/
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_EncoderTask */
/**
* @brief Function implementing the EncoderContol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderTask */
void EncoderTask(void const * argument)
{
  /* USER CODE BEGIN EncoderTask */
  /* Infinite loop */
  for(;;)
  {
	 ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  if(counter != temp)
	  	      {
		  	    angle = (float)counter * ANGLE_PER_PULSE / PULSES_PER_REV;
	  	        sprintf(buf, "Counter: %ld\n", counter); // Send counter value over serial
	  	        HAL_UART_Transmit(&hlpuart1, (uint8_t *)buf, strlen(buf), 100);
	  	      sprintf(buffer, "Angle: %f degree\r\n", angle);
	  	         HAL_UART_Transmit(&hlpuart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	  	        temp = counter; // Update temporary variable with current counter value
	  	      }
    osDelay(1);
  }
  /* USER CODE END EncoderTask */
}

/* USER CODE BEGIN Header_PIDControllerTask */
/**
* @brief Function implementing the PIDContol thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PIDControllerTask */
void PIDControllerTask(void const * argument)
{
  /* USER CODE BEGIN PIDControllerTask */
  /* Infinite loop */
  for(;;)
  {
	  if(setpoint)
	  {
		  HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
	  }
	  else
	  {
	  	  HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
	  }
	  if (xSemaphoreTakeFromISR(myBinarySem01Handle, portMAX_DELAY) == pdTRUE)
	  {
	  // Read the feedback value from the encoder and convert it to angle */
	  //semtake
	  feedback = ((float)counter / 1600) * 360;


	  /* Calculate the error between the setpoint and feedback */
	  error = setpoint - feedback;

	  /* Calculate the integral error over time */
	  integral += error;

	  /* Limit the integral error to avoid windup */
	  if (integral > windup_guard)
	  {
	    integral = windup_guard;
	  }
	  else if (integral < -windup_guard)
	  {
	    integral = -windup_guard;
	  }

	  /* Calculate the derivative error over time */
	  if (proportional_on_measurement)
	  {
	    derivative = feedback - last_feedback;
	  }
	  else
	  {
	    derivative = error - last_error;
	  }

	  /* Calculate the output value to the actuator using PID algorithm */
	  output = Kp * error + Ki * integral + Kd * derivative;

	  /* Send the output value to the actuator */
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, output);

	  /* Save the error and feedback values for the next iteration */
	  last_error = error;

	  /* Send the value of counter in angle */
	  sprintf(buff,"Angle: %.2f\r\n", feedback);
	  HAL_UART_Transmit(&hlpuart1,(uint8_t)buff, strlen(buff), 500);

	  if (output >= 0)
	          {
	              HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
	              HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
	          }
	  else
	          {
	              HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
	              HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
	          }



	  /*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
	     osDelay(1000);
	     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET);
	     osDelay(1000);*/
    //osDelay(1);
  }//semgive

  }
  /* USER CODE END PIDControllerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
