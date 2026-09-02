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
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drivers/ad4130_measurement.h"
#include "application/sensor_adc.h"
#include "application/sensor_coeffs.h"
#include "application/sensor_fit.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_READ_OK         0
#define SENSOR_READ_ADC_ERROR -1
#define SENSOR_READ_FIT_ERROR -2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float resistance[2][4];
static float temperature[2][4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int read_sensor(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int read_sensor(void)
{
	HAL_StatusTypeDef result;
	uint8_t channel;
	int fit_result;
	float measured_resistance;
	float measured_temperature;

	for (uint8_t i = 0; i < 2U; i++)
	{
		result = AD4130_Read_Resistance(i+1U, &channel, &measured_resistance);
		if (result == HAL_BUSY)
		{
			HAL_Delay(1);
			continue;
		}
		if (result != HAL_OK)
		{
			printf(
					"ADC %u read incorrect: %d\r\n",
					(unsigned int)(i+1U),
					(int)result
			);
			return SENSOR_READ_ADC_ERROR;
		}

		/* sensor_fit: temperature fit */
		fit_result = resistance_to_temperature(
				measured_resistance,
				sensor_coeffs_get_curve(i+1U, channel),
				&measured_temperature
		);
		if (fit_result != SENSOR_FIT_OK)
		{
			printf(
					"ADC %u CHANNEL_%u fit incorrect: %d\r\n",
					(unsigned int)(i+1U),
					(unsigned int)channel,
					fit_result
			);
			return SENSOR_READ_FIT_ERROR;
		}

		resistance[i][channel] = measured_resistance;
		temperature[i][channel] = measured_temperature;

		printf(
				"%u-%u: R-%.4f ohm, T-%.5f K\r\n",
				(unsigned int)(i+1U),
				(unsigned int)(channel+1U),
				(double)resistance[i][channel],
				(double)temperature[i][channel]
		);
		HAL_Delay(1000);
	}

	return SENSOR_READ_OK;
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	int adc_result;
	int coeffs_result;
	int read_result;


	/* sensor_adc: ADC initialize */
	adc_result = sensor_adc_initialize();
	if (adc_result != SENSOR_ADC_OK)
	{
		Error_Handler();
	}

	/* sensor_coeffs: load, decode */
	coeffs_result = sensor_coeffs_initialize();
	if (coeffs_result != SENSOR_COEFFS_OK)
	{
		Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* sensor_coeffs: transfer, decode, save */
		coeffs_result = sensor_coeffs_process();
		if (coeffs_result == SENSOR_COEFFS_UPDATED)
		{
			NVIC_SystemReset();
		}
		else if (coeffs_result != SENSOR_COEFFS_NOT_READY)
		{
			Error_Handler();
		}

		/* ADC read */
		read_result = read_sensor();
		if (read_result != SENSOR_READ_OK)
		{
			Error_Handler();
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
	printf(
		"Wrong parameters value: file %s on line %lu\r\n",
		(char *)file, (unsigned long int)line
	);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
