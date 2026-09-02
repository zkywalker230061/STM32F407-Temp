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
#include "communication/sensor_coeffs_transfer.h"
#include "storage/sensor_coeffs_decoder.h"
#include "storage/sensor_coeffs_storage.h"
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
static Curve_t sensor_curves[2][4];
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
  MX_USB_DEVICE_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	/* -------------------------------------------------------------------- */

	HAL_StatusTypeDef result;
	AD4130InitResult_t init_result[2] = {0};
	uint8_t adc_device_id;
	uint8_t channel_id;
	const uint8_t *binary_data;
	uint32_t binary_length;
	Curve_t *received_curve;
	int transfer_result;
	int decode_result;
	int storage_result;
	float resistance[2][4] = {0};
	float temperature[2][4] = {0};

	/* -------------------------------------------------------------------- */

	/* storage load */
	storage_result = Sensor_Coeffs_Storage_Load(
			&adc_device_id,
			&channel_id,
			&binary_data,
			&binary_length
	);
	if (storage_result == SENSOR_COEFFS_STORAGE_OK)
	{
		/* decode */
		received_curve = &sensor_curves[adc_device_id - 1U][channel_id];
		decode_result = Sensor_Coeffs_Decode(
				binary_data,
				binary_length,
				received_curve
		);
		if (decode_result != SENSOR_COEFFS_DECODE_OK)
		{
			printf(
					"Stored sensor coefficients decode error: %d\r\n",
					decode_result
			);
		}
		else
		{
			printf(
					"ADC %u CHANNEL_%u: %u segments loaded\r\n",
					(unsigned int)adc_device_id,
					(unsigned int)channel_id,
					(unsigned int)received_curve->segment_count
			);
		}
	}
	else if (storage_result != SENSOR_COEFFS_STORAGE_NOT_FOUND)
	{
		printf(
				"Sensor coefficients storage load error: %d\r\n",
				storage_result
		);
	}

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

		/* ADC_CONTROL:		0x2700 - 0010 0111 0000 0000 */
		/* IO_CONTROL:		0x0000 - 0000 0000 0000 0000 */
		/* VBIAS_CONTROL:	0x0000 - 0000 0000 0000 0000 */
		/* ERROR_EN:		0x0078 - 0000 0000 0111 1000 */

	}

	for (uint8_t i = 0; i < 2U; i++)
	{
		result = AD4130_Channel_0(i+1U, 2U);  /* I_OUT0_0 */
		if (result != HAL_OK)
		{
			printf("ADC %u CHANNEL_0 setup incorrect\r\n", (unsigned int)(i+1U));
			Error_Handler();
		}

//		result = AD4130_Channel_1(i+1U, 2U);
//		if (result != HAL_OK)
//		{
//			printf("ADC %u CHANNEL_1 setup incorrect\r\n", (unsigned int)(i+1U));
//			Error_Handler();
//		}
//
//		result = AD4130_Channel_2(i+1U, 2U);
//		if (result != HAL_OK)
//		{
//			printf("ADC %u CHANNEL_2 setup incorrect\r\n", (unsigned int)(i+1U));
//			Error_Handler();
//		}
//
//		result = AD4130_Channel_3(i+1U, 2U);
//		if (result != HAL_OK)
//		{
//			printf("ADC %u CHANNEL_3 setup incorrect\r\n", (unsigned int)(i+1U));
//			Error_Handler();
//		}
	}

	/* -------------------------------------------------------------------- */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* transfer */
		transfer_result = Sensor_Coeffs_Transfer_Get_Data(
				&adc_device_id,
				&channel_id,
				&binary_data,
				&binary_length
		);
		if (transfer_result == SENSOR_COEFFS_TRANSFER_OK)
		{
			/* decode */
			received_curve = &sensor_curves[adc_device_id - 1U][channel_id];
			decode_result = Sensor_Coeffs_Decode(
					binary_data,
					binary_length,
					received_curve
			);
			if (decode_result != SENSOR_COEFFS_DECODE_OK)
			{
				printf(
						"Sensor coefficients decode error: %d\r\n",
						decode_result
				);
				Sensor_Coeffs_Transfer_Reset();
				continue;
			}

			/* storage save */
			storage_result = Sensor_Coeffs_Storage_Save(
					adc_device_id,
					channel_id,
					binary_data,
					binary_length
			);
			if (storage_result != SENSOR_COEFFS_STORAGE_OK)
			{
				printf(
						"Sensor coefficients storage save error: %d\r\n",
						storage_result
				);
				Sensor_Coeffs_Transfer_Reset();
				continue;
			}

			printf(
					"ADC %u CHANNEL_%u: %u segments saved\r\n",
					(unsigned int)adc_device_id,
					(unsigned int)channel_id,
					(unsigned int)received_curve->segment_count
			);
			NVIC_SystemReset();
		}
		if (transfer_result != SENSOR_COEFFS_TRANSFER_NOT_READY)
		{
			printf(
					"Sensor coefficients transfer error: %d\r\n",
					transfer_result
			);
			Sensor_Coeffs_Transfer_Reset();
			continue;
		}

		for (uint8_t i = 0; i < 2U; i++)
		{
			uint8_t channel;
			int fit_result;
			float measured_resistance;
			float measured_temperature;

			result = AD4130_Read_Resistance(i+1U, &channel, &measured_resistance);
			if (result == HAL_BUSY)
			{
				HAL_Delay(1);
				continue;
			}
			if (result != HAL_OK)
			{
				printf("ADC %u read incorrect\r\n", (unsigned int)(i+1U));
				Error_Handler();
			}

			fit_result = resistance_to_temperature(
					measured_resistance,
					&measured_temperature
			);
			if (fit_result != SENSOR_FIT_OK)
			{
				printf(
						"ADC %u CHANNEL_%u fit incorrect\r\n",
						(unsigned int)(i+1U),
						(unsigned int)channel
				);
				Error_Handler();
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
