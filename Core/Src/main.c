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
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drivers/ad4130.h"
#include "drivers/ad4130_measurement.h"
#include "application/sensor_fit.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
	HAL_StatusTypeDef result;
	AD4130InitResult_t init_result[2] = {0};

	for (uint8_t i = 0; i < 2U; i++)
	{
		result = AD4130_Init(i+1U, &init_result[i]);
		if (result != HAL_OK)
		{
			printf("ADC %u INIT incorrect\r\n", (unsigned int)(i+1U));
			Error_Handler();
		}

		printf(
			"ADC %u\r\n"
			"ID: 0x%02X\r\n"
			"STATUS: 0x%02X\r\n"
			"ERROR: 0x%04X\r\n",
			(unsigned int)(i+1U),
			/*ID: 0x05 - 0000 0101 */
			(unsigned int)init_result[i].id,
			/* STATUS (init): 0x90 - 1001 0000 or 0x10 - 0001 0000 */
			(unsigned int)init_result[i].status,
			/* ERROR: 0x0000 - 0000 0000 0000 0000 */
			(unsigned int)init_result[i].error
		);

		if ((init_result[i].id != 0x05U) || (init_result[i].error != 0x0000U))
		{
			printf("ADC %u ID or ERROR incorrect\r\n", (unsigned int)(i+1U));
			Error_Handler();
		}

		/* ADC_CONTROL: 0x2700 - 0010 0111 0000 0000 */
		/* IO_CONTROL: 0x0000 - 0000 0000 0000 0000 */
		/* VBIAS_CONTROL: 0x0000 - 0000 0000 0000 0000 */
		/* ERROR_EN: 0x0078 - 0000 0000 0111 1000 */

	}

	for (uint8_t i = 1; i < 2U; i++)
	{
		result = AD4130_Channel_0(i+1U, 2U);  /* I_OUT0_0 */
		if (result != HAL_OK)
		{
			printf("ADC %u CHANNEL_0 setup incorrect\r\n", (unsigned int)(i+1U));
			Error_Handler();
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		for (uint8_t i = 1; i < 2U; i++)
		{
			int fit_result;
			float r_2_1;
			float temperature_2_1;

			result = AD4130_Read_Channel_0_Resistance(i+1U, &r_2_1);
			if (result == HAL_BUSY)
			{
				HAL_Delay(1);
				continue;
			}
			if (result != HAL_OK)
			{
				printf("ADC %u CHANNEL_0 read incorrect\r\n", (unsigned int)(i+1U));
				Error_Handler();
			}
			printf("Resistance: %.4f ohm\r\n", (double)r_2_1);

			fit_result = resistance_to_temperature(r_2_1, &temperature_2_1);
			if (fit_result != SENSOR_FIT_OK)
			{
				printf("ADC %u CHANNEL_0 fit incorrect\r\n", (unsigned int)(i+1U));
				Error_Handler();
			}
			printf("Temperature: %.5f K\r\n", (double)temperature_2_1);

			HAL_Delay(10000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
