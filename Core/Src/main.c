/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <errno.h>
#include <stdio.h>

#include "bsp_oled.h"
#include "bsp_dwt.h"
#include "bsp_adc.h"

#include "esc_telemetry_stm32.h"
#include "hall_speed.h"
#include "servo_basic_control.h"

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

/* USER CODE BEGIN PV */

uint8_t rosbuffer = 0;

DebugType_t g_sys_debug = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void App_InitRuntimeServices(void);
static pOLEDInterface_t App_InitDisplay(void);
static void App_DetectHardwareVersion(pOLEDInterface_t oled);
static void App_HandleUnknownHardware(pOLEDInterface_t oled);
static void App_StartInputInterrupts(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_InitRuntimeServices(void)
{
  ServoBasic_Init();
  EscTelemetryStm32_Init();
  DWT_Init();
  HallSpeed_Init();
  ADC_Userconfig_Init();
  HAL_TIM_Base_Start(&htim6);
}

static pOLEDInterface_t App_InitDisplay(void)
{
  pOLEDInterface_t oled = &UserOLED;
  oled->init();
  return oled;
}

static void App_DetectHardwareVersion(pOLEDInterface_t oled)
{
  uint8_t version = 0U;

  version |= (uint8_t)(HAL_GPIO_ReadPin(VersionBit2_GPIO_Port, VersionBit2_Pin) << 2);
  version |= (uint8_t)(HAL_GPIO_ReadPin(VersionBit1_GPIO_Port, VersionBit1_Pin) << 1);
  version |= (uint8_t)(HAL_GPIO_ReadPin(VersionBit0_GPIO_Port, VersionBit0_Pin) << 0);

  if (version == 0U || version == 7U)
  {
    return;
  }

  App_HandleUnknownHardware(oled);
}

static void App_HandleUnknownHardware(pOLEDInterface_t oled)
{
  oled->ShowString(0, 0, "unknown hardware-version.");
  oled->ShowString(0, 35, "please reset and-retry.");
  oled->RefreshGram();

  for (;;)
  {
    HAL_GPIO_TogglePin(UserBuzzer_GPIO_Port, UserBuzzer_Pin);
    HAL_Delay(500);
  }
}

static void App_StartInputInterrupts(void)
{
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart4, &rosbuffer, 1);
  (void)EscTelemetryStm32_Start();
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
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  pOLEDInterface_t oled = NULL;

  App_InitRuntimeServices();
  oled = App_InitDisplay();
  App_DetectHardwareVersion(oled);
  App_StartInputInterrupts();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* USER CODE BEGIN 4 */

UART_HandleTypeDef *DebugSerial = &huart1;

static int DebugSerial_Write(const uint8_t *data, size_t length)
{
  size_t written = 0U;

  if (data == NULL && length != 0U)
  {
    errno = EINVAL;
    return -1;
  }

  while (written < length)
  {
    size_t chunk = length - written;

    if (chunk > UINT16_MAX)
    {
      chunk = UINT16_MAX;
    }

    if (HAL_UART_Transmit(DebugSerial,
                          &data[written],
                          (uint16_t)chunk,
                          100U) != HAL_OK)
    {
      if (written == 0U)
      {
#ifdef EIO
        /* ARMCC 5 does not define the POSIX I/O error code. */
        errno = EIO;
#endif
        return -1;
      }
      break;
    }

    written += chunk;
  }

  return (int)written;
}

//printf鍑芥暟瀹炵幇
int fputc(int ch, FILE *stream)
{
  uint8_t byte = (uint8_t)ch;

  (void)stream;
  while (DebugSerial_Write(&byte, 1U) != 1)
  {
  }
  return ch;
}

#if defined(__GNUC__)
int _write(int fd, char *ptr, int len)
{
  if (fd != 1 && fd != 2)
  {
    errno = EBADF;
    return -1;
  }

  if (len < 0)
  {
    errno = EINVAL;
    return -1;
  }

  return DebugSerial_Write((const uint8_t *)ptr, (size_t)len);
}
#endif

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
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
