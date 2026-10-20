/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "app_motor.h"
#include "app_servo.h"
#include "app_imu.h"
#include "app_encoder.h"
#include "app_motion.h"
#include "app_comm.h"
#include "app_ultrasonic.h"
#include "app_park_trace.h"
#include "app_ir.h"
#include "oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define APP_STATUS_PERIOD_TICKS 500U

/* IMU cold-boot recovery (StartIMUTask): named in RTOS ticks, matching
 * osDelay()'s actual parameter and this file's own APP_STATUS_PERIOD_TICKS
 * convention above, not renamed to "_MS" to avoid implying osDelay() takes
 * milliseconds. configTICK_RATE_HZ is 1000 (FreeRTOSConfig.h), so 1 tick
 * is exactly 1ms here and these values are numerically 500ms/1000ms, but
 * the names intentionally track the unit osDelay() itself consumes.
 *
 * IMU_POWERUP_SETTLE_TICKS: one-time delay before the very first
 * AppIMU_Init() probe of the cycle, to let the ICM-20948 (and any board-
 * level power-rail sequencing) reach a stable, I2C-responsive state after
 * a cold power-on. This is separate from, and does not replace, the
 * existing post-init gyro-settling delay below -- that one lets the
 * already-initialized/already-reset gyro's output settle before bias
 * calibration; this one is about the sensor being electrically present on
 * the bus at all before the first probe is even attempted.
 *
 * IMU_STARTUP_RETRY_INTERVAL_TICKS: delay before retrying after EITHER a
 * failed AppIMU_Init() or a failed AppIMU_Calibrate() -- one shared
 * constant for both, since ERR,IMU cannot and does not need to distinguish
 * which of the two failed. Long enough that retries never spin rapidly or
 * flood anything; short enough that recovery completes in a few seconds
 * even after several retries. */
#define IMU_POWERUP_SETTLE_TICKS         500U
#define IMU_STARTUP_RETRY_INTERVAL_TICKS 1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osMessageQueueId_t MotionQueueHandle;
osMessageQueueId_t CommRxQueueHandle;

/* Ultrasonic ranging task. Created by hand (not in RTOSV.ioc) so CubeMX
 * regeneration leaves it alone. Priority is BelowNormal so it can never
 * delay MotionTask or CommTask (the STOP path); edge timing is done by TIM8
 * hardware, so task latency only affects measurement cadence and age.
 * The stack is 512 bytes (128 words); stack_size is in bytes. */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

#if APP_IR_ENABLE
/* IR raw acquisition task (APP_IR_ENABLE, default 0). Created by hand like
 * UltrasonicTask so CubeMX regeneration leaves it alone; same BelowNormal
 * priority, so it can never delay MotionTask or CommTask. 512-byte stack. */
osThreadId_t IRTaskHandle;
const osThreadAttr_t IRTask_attributes = {
  .name = "IRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
#endif
#if APP_IR_ENABLE
/* MotionTask's copy of the newest raw IR publication, for the Phase 5 edge
 * detector (MOV,FIRL / MOV,FIRR). A file static keeps it off MotionTask's
 * small stack. */
static AppIrSnapshot motionIrSnapshot;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void *argument);
void StartMotionTask(void *argument);
void StartIMUTask(void *argument);
void StartEncoderTask(void *argument);
void StartCommTask(void *argument);

/* USER CODE BEGIN PFP */
void StartUltrasonicTask(void *argument);
#if APP_IR_ENABLE
void StartIRTask(void *argument);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t AppUI_Magnitude(int32_t value)
{
  if (value < 0)
  {
    return (uint32_t)(-(value + 1)) + 1U;
  }

  return (uint32_t)value;
}

static int32_t AppUI_ToTenths(float value)
{
  float scaled = value * 10.0f;

  if (scaled >= 0.0f)
  {
    scaled += 0.5f;
  }
  else
  {
    scaled -= 0.5f;
  }

  return (int32_t)scaled;
}

static void AppUI_ClearBuffer(void)
{
  uint8_t page;
  uint8_t x;

  for (page = 0U; page < (OLED_Height / 8U); page++)
  {
    for (x = 0U; x < OLED_Width; x++)
    {
      OLED_GRAM[x][page] = 0U;
    }
  }
}

static void AppUI_ShowSigned(
    uint8_t x,
    uint8_t y,
    int32_t value,
    uint8_t digits)
{
  OLED_ShowChar(
      x,
      y,
      (uint8_t)((value < 0) ? '-' : ' '),
      12U,
      1U
  );
  OLED_ShowNumber(
      (uint8_t)(x + 6U),
      y,
      AppUI_Magnitude(value),
      digits,
      12U
  );
}

static void AppUI_ShowHeadingX10(
    uint8_t x,
    uint8_t y,
    int32_t headingX10)
{
  uint32_t magnitude = AppUI_Magnitude(headingX10);

  OLED_ShowChar(
      x,
      y,
      (uint8_t)((headingX10 < 0) ? '-' : ' '),
      12U,
      1U
  );
  OLED_ShowNumber(
      (uint8_t)(x + 6U),
      y,
      magnitude / 10U,
      6U,
      12U
  );
  OLED_ShowChar(
      (uint8_t)(x + 42U),
      y,
      (uint8_t)'.',
      12U,
      1U
  );
  OLED_ShowNumber(
      (uint8_t)(x + 48U),
      y,
      magnitude % 10U,
      1U,
      12U
  );
}

static void AppUI_Refresh(void)
{
  AppEncoderSnapshot encoder = AppEncoder_GetSnapshot();
  AppIMUSnapshot imu;
  AppMotionState state = AppMotion_GetState();

  AppIMU_GetSnapshot(&imu);
  AppUI_ClearBuffer();

  OLED_ShowString(0U, 0U, (const uint8_t *)"S:");
  OLED_ShowNumber(16U, 0U, (uint32_t)state, 2U, 12U);
  OLED_ShowString(40U, 0U, (const uint8_t *)"I:");
  OLED_ShowNumber(56U, 0U, (uint32_t)imu.ready, 1U, 12U);

  OLED_ShowString(0U, 16U, (const uint8_t *)"L:");
  AppUI_ShowSigned(16U, 16U, encoder.left, 10U);

  OLED_ShowString(0U, 32U, (const uint8_t *)"R:");
  AppUI_ShowSigned(16U, 32U, encoder.right, 10U);

  OLED_ShowString(0U, 48U, (const uint8_t *)"H:");

  if (imu.ready != 0U)
  {
    AppUI_ShowHeadingX10(
        16U,
        48U,
        AppUI_ToTenths(imu.headingDeg)
    );
  }
  else
  {
    OLED_ShowString(16U, 48U, (const uint8_t *)"ERR");
  }

  OLED_Refresh_Gram();
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
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  MotionQueueHandle = osMessageQueueNew(
      8,
      sizeof(MotionCommand),
      NULL
  );

  if (MotionQueueHandle == NULL)
  {
      Error_Handler();
  }

  CommRxQueueHandle = osMessageQueueNew(
      128,
      sizeof(uint8_t),
      NULL
  );

  if (CommRxQueueHandle == NULL)
  {
      Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &EncoderTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* Soft-fail by design: a failed creation must never hang the boot of the
   * motion/comm tasks (Error_Handler() disables interrupts forever). It is
   * not silent, though: the consumer-visible snapshot becomes FAULT
   * (faultMask APP_US_FAULT_TASK, also shown as ST:FAULT/FM:80 in the bench
   * log), PC6 is forced low and TIM8 capture is never started. Servicing is
   * NOT moved into another task. */
  UltrasonicTaskHandle = osThreadNew(StartUltrasonicTask, NULL, &UltrasonicTask_attributes);

  if (UltrasonicTaskHandle == NULL)
  {
    AppUltrasonic_ReportTaskCreateFailure();
  }

#if APP_IR_ENABLE
  /* Same soft-fail policy: the IR snapshot becomes a latched FAULT
   * (APP_IR_FAULT_TASK) and the ADC is never started. */
  IRTaskHandle = osThreadNew(StartIRTask, NULL, &IRTask_attributes);

  if (IRTaskHandle == NULL)
  {
    AppIr_ReportTaskCreateFailure();
  }
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 167;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 99;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 9999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1530;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RST_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : US_TRIG_Pin */
  GPIO_InitStruct.Pin = US_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* TIM8 callback dispatchers for the ultrasonic driver. Strong definitions
 * of the HAL's weak callbacks; this file is their single owner so every
 * TIM routing decision is visible in one place. They run inside
 * HAL_TIM_IRQHandler() (TIM8_CC_IRQHandler), which has already cleared the
 * channel flag and set htim->Channel. USE_HAL_TIM_REGISTER_CALLBACKS is 0.
 *   CH2 input capture  -> Echo edge timestamp
 *   CH3 output compare -> end of Trigger pulse, then no-Echo timeout
 * The HAL also calls HAL_TIM_PWM_PulseFinishedCallback() for CH3; that
 * stays the (empty) weak default. TIM6's HAL_TIM_PeriodElapsedCallback()
 * above is a different function and is unaffected. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM8) &&
      (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
  {
    AppUltrasonic_OnCapture();
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM8) &&
      (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
  {
    AppUltrasonic_OnCompare();
  }
}

/**
  * @brief  UltrasonicTask: triggers, classifies, filters and publishes.
  *         Fixed 10 ms period; never blocks on the sensor (edge timing is
  *         done by TIM8 hardware). Readings are NOT connected to motion
  *         control or the stopping logic.
  */
void StartUltrasonicTask(void *argument)
{
  uint32_t nextWake;

  (void)argument;

  AppUltrasonic_Init();
  nextWake = osKernelGetTickCount();

  for (;;)
  {
    AppUltrasonic_Service(osKernelGetTickCount());

    nextWake += APP_US_SERVICE_PERIOD_TICKS_RTOS;
    (void)osDelayUntil(nextWake);
  }
}

#if APP_IR_ENABLE
/* ADC1 callback dispatchers for the IR acquisition (APP_IR_ENABLE). Strong
 * definitions of the HAL's weak callbacks; this file is their single owner.
 * They run inside DMA2_Stream0_IRQHandler() -> HAL_DMA_IRQHandler() (priority
 * 5, masked by FreeRTOS critical sections). ADC_IRQn is not enabled, so the
 * error callback is reached only through the HAL's DMA error path.
 * USE_HAL_ADC_REGISTER_CALLBACKS is 0. HAL_ADC_ConvHalfCpltCallback() stays
 * the (empty) weak default. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    AppIr_OnConvComplete();
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    AppIr_OnError(hadc->ErrorCode);
  }
}

/**
  * @brief  IRTask: raw two-channel IR ADC acquisition (Phase 1). Fixed
  *         10 ms service period; one scan in flight at most, never waits
  *         on the ADC. Raw counts only -- NOT distance, NOT connected to
  *         motion control or the stopping logic.
  */
void StartIRTask(void *argument)
{
  uint32_t nextWake;

  (void)argument;

  AppIr_Init();
  nextWake = osKernelGetTickCount();

  for (;;)
  {
    AppIr_Service(osKernelGetTickCount());

    nextWake += APP_IR_SERVICE_PERIOD_TICKS;
    (void)osDelayUntil(nextWake);
  }
}
#endif /* APP_IR_ENABLE */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  uint32_t nextWake;

  (void)argument;
  (void)osThreadSetPriority(osThreadGetId(), osPriorityLow);

  OLED_Init();
  nextWake = osKernelGetTickCount();

  for(;;)
  {
    AppUI_Refresh();
    HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);

    nextWake += APP_STATUS_PERIOD_TICKS;
    (void)osDelayUntil(nextWake);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotionTask */
/**
* @brief Function implementing the MotionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotionTask */
void StartMotionTask(void *argument)
{
  /* USER CODE BEGIN StartMotionTask */

  MotionCommand command;

  AppServo_Init();
  AppMotor_Init();
  AppMotion_Init();

  uint32_t lastTick = osKernelGetTickCount();

  for (;;)
  {
    /* Priority STOP first, then the normal queue (see
     * AppComm_TakeMotionCommand() in app_comm.c). */
    if (AppComm_TakeMotionCommand(&command))
    {
      AppMotion_HandleCommand(&command);
    }

    uint32_t now =
        osKernelGetTickCount();

    float dtSeconds =
        (float)(now - lastTick)
        / (float)osKernelGetTickFreq();

    lastTick = now;

#if APP_IR_ENABLE
    /* Phase 5: while a MOV,FIRL / MOV,FIRR movement is armed, hand it the
     * newest raw IR publication. Before AppMotion_Update() and outside it, so
     * the IR call chain never nests under Update's large frame. A plain MOV
     * makes no IR call at all. */
    if (AppMotion_IrEdgeArmed())
    {
        AppIr_GetSnapshot(&motionIrSnapshot);
        AppMotion_ServiceIrEdge(&motionIrSnapshot, now);
    }
#endif

    AppMotion_Update(dtSeconds);

#if APP_USPARK_TRACE
    /* Measurement-only USPARK trace: both encoders, every cycle; the trace
     * ignores it unless an automatic parking stop window is open. */
    AppParkTrace_Sample(osKernelGetTickCount(),
                        AppEncoder_Left(),
                        AppEncoder_Right());
#endif

    osDelay(10);
  }

  /* USER CODE END StartMotionTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */

  /* Cold-boot IMU recovery: the leading hypothesis is a power-up race
   * between the MCU (which can finish its own boot and reach this task
   * within milliseconds) and the external ICM-20948 sensor's own power-on/
   * reset time, matching the observed symptom (fails after cold power-on,
   * recovers after a warm STM reset that never removed the sensor's own
   * power). This is NOT proven, though: ERR,IMU (app_comm.c) only reports
   * whether the published IMU snapshot is ready, and cannot distinguish an
   * AppIMU_Init() probe/configuration failure from an AppIMU_Calibrate()
   * failure. The retry loop below does not assume which one occurs -- it
   * covers both uniformly by always restarting from a full AppIMU_Init()
   * after any failure, so it recovers correctly regardless of where the
   * real fault lies.
   *
   * All initialization/recovery ownership stays here, inside IMUTask,
   * exactly as before; on_vision_ready() (app_comm.c) remains empty and
   * unrelated. No MCU reset is performed -- only this task's own retry
   * loop. */
  osDelay(IMU_POWERUP_SETTLE_TICKS);

  for (;;)
  {
    if (!AppIMU_Init())
    {
      /* imuReady remains false throughout (AppIMU_Init() itself forces
       * it false) -- wait, then retry. Never spins without a delay. */
      osDelay(IMU_STARTUP_RETRY_INTERVAL_TICKS);
      continue;
    }

    /* Let gyro settle before bias calibration -- unchanged, still runs
     * after every successful AppIMU_Init(), including retries. */
    osDelay(1000);

    if (AppIMU_Calibrate())
    {
      break;
    }

    /* Complete any request accepted before startup calibration failed. */
    (void)AppIMU_ServiceRequest();

    /* Calibration failed: retry from a full AppIMU_Init() (not a bare
     * AppIMU_Calibrate() retry) so the driver and sensor return to a
     * known clean state -- re-probes the device, re-issues its internal
     * reset, and re-establishes every app-layer static, rather than
     * assuming the prior configuration/state survived whatever caused
     * the calibration read to fail. */
    osDelay(IMU_STARTUP_RETRY_INTERVAL_TICKS);
  }

  uint32_t lastTick = osKernelGetTickCount();

  for (;;)
  {
    if (AppIMU_ServiceRequest())
    {
      /* Exclude request servicing/calibration time from heading integration. */
      lastTick = osKernelGetTickCount();
    }

    uint32_t now = osKernelGetTickCount();

    float dt = (float)(now - lastTick)
             / (float)osKernelGetTickFreq();

    lastTick = now;

    AppIMU_Update(dt);

    osDelay(10);
  }

  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartEncoderTask */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoderTask */
void StartEncoderTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderTask */

  AppEncoder_Init();

  for (;;)
  {
    AppEncoder_Update();

    osDelay(10);
  }

  /* USER CODE END StartEncoderTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */

  uint8_t byte;

  AppComm_Init();

  for (;;)
  {
    if (osMessageQueueGet(
          CommRxQueueHandle,
          &byte,
          NULL,
          10U) == osOK)
    {
      AppComm_ProcessByte(byte);
    }

    AppComm_ServiceTelemetry();

    /* Ultrasonic parking protection (opt-in, off after boot): evaluated
     * here, in CommTask, and requests the same priority STOP as the manual
     * command. Returns immediately while disabled. */
    AppComm_ServiceParking();

#if APP_USPARK_TRACE
    /* Measurement-only USPARK trace (bench builds): sends at most one line
     * per pass, and only once the stop is over and the link is idle. */
    AppComm_ServiceParkTrace();
#endif
  }

  /* USER CODE END StartCommTask */
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
  if (htim->Instance == TIM6)
  {
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
#ifdef USE_FULL_ASSERT
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
