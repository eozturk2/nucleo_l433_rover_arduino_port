/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/*
 * TODO: Pinouts
 *
 * [SPI Bus (SPI2)]
 * PC2 - MISO
 * PC3 - MOSI
 * PB10 - SCLK
 *
 * [Motor #1]
 * PA1 - PWM1
 * PC13 - DIR1
 * PB11 - CS1
 * Limit switch pin(s)?
 *
 * [Motor #2]
 * PA2 - PWM2
 * PB13 - DIR2
 * PA4 - CS2
 * Limit switch pin(s)?
 *
 * [Motor #3]
 * PA3 - PWM3
 * PA8 - DIR3
 * PB12 - CS3
 * Limit switch pin(s)?
 *
 * [Motor #4]
 * PA0 - PWM4
 * (No direction pin)
 * (No chip select pin)
 * Limit switch pin(s)?
 *
 */

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RoverArmMotor.h"
#include "AMT22.h"
//Standard includes
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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
void print(const char* s){
//	#ifdef PRINT
	HAL_StatusTypeDef code = HAL_UART_Transmit(&huart2, (uint8_t*) s, strlen(s), HAL_MAX_DELAY);
//	#endif
}
int printf(const char* s, ...){
	char buffer[256];
//	#ifdef PRINT
	va_list args;
	va_start(args, s);
	vsprintf(buffer, s, args);
	perror(buffer);
	print(buffer);
	va_end(args);
//	#endif
	return strlen(buffer);
}
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  int16_t encoderData_1 = 99;
  int16_t encoderTurns_1 = 0;

  int16_t encoderData_2 = 99;
  int16_t encoderTurns_2 = 0;

  int16_t encoderData_3 = 99;
  int16_t encoderTurns_3 = 0;


  // TODO: Initialize other PWM modules to expand functionality to
  // different motors with different pins
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  

  // TODO: Figure out how to reset the turn counter at powerup. Right now it just holds onto the previous
  // value and resetting/setting to zero doesn't seem to help.
  resetAMT22(&hspi1, GPIOC, GPIO_PIN_7, &htim1);
  setZeroSPI(&hspi1, GPIOC, GPIO_PIN_7, &htim1);

  // Reverse direction
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t cycle = 0;

  // Meant to sweep through speeds and print out the position and turn counter. You can go to
  // Device Manager > COM Ports to find the Nucleo, then open PuTTY or any other terminal emulator
  // you like, then set the Port = Nucleo's port and Baud rate = 115200.
  int16_t pos[2];

  while (1)
  {
	  while((cycle < 100))
	  {
		  // PWM cycle set function: 0-100 maps to 0-100% duty cycle
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, cycle);

		  // TODO: Handle the case where the checksum fails and the return value is 0xFFFF. What is it
		  // supposed to do?
		  getTurnCounterSPI(pos, &hspi1, GPIOC, GPIO_PIN_7, 12, &htim1);

		  // TODO: Convert an [angle, turn counter] array into a useful measure of absolute angle
		  encoderData_1 = pos[0];
		  encoderTurns_1 = pos[1];
		  // encoderData_2 = getPositionSPI(&hspi2, GPIOB, GPIO_PIN_6, 12, &htim1);
		  // encoderData_3 = getPositionSPI(&hspi3, GPIOA, GPIO_PIN_8, 12, &htim1);

		  printf("encoder 1 gives %d\r\n", encoderData_1);
		  printf("encoder 1 turns %d\r\n", encoderTurns_1);
//		  printf("encoderData_1: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\r\n",
//				  BYTE_TO_BINARY(encoderData_1>>24), BYTE_TO_BINARY(encoderData_1>>16), BYTE_TO_BINARY(encoderData_1>>8), BYTE_TO_BINARY(encoderData_1));
//
//		  printf("encoderTurns_1: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\r\n",
//				  BYTE_TO_BINARY(encoderTurns_1>>8), BYTE_TO_BINARY(encoderTurns_1));
		  cycle--;
		  HAL_Delay(100);
	  }

	  while(cycle > 0)
	  {
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, cycle);
		  getTurnCounterSPI(pos, &hspi1, GPIOC, GPIO_PIN_7, 12, &htim1);
		  encoderTurns_1 = pos[1];
		  // encoderData_1 = pos[0];
		  // encoderTurns_1 = pos[1];
		  // encoderData_2 = getPositionSPI(&hspi2, GPIOB, GPIO_PIN_6, 12, &htim1);
		  // encoderData_3 = getPositionSPI(&hspi3, GPIOA, GPIO_PIN_8, 12, &htim1);

		  printf("encoder 1 gives %d\r\n", pos[0]);
		  printf("encoder 1 turns %d\r\n", encoderTurns_1);
//		  printf("encoderData_1: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\r\n",
//				  BYTE_TO_BINARY(encoderData_1>>24), BYTE_TO_BINARY(encoderData_1>>16), BYTE_TO_BINARY(encoderData_1>>8), BYTE_TO_BINARY(encoderData_1));
//
//		  printf("encoderTurns_1: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\r\n",
//				  BYTE_TO_BINARY(encoderTurns_1>>8), BYTE_TO_BINARY(encoderTurns_1));
		  cycle--;
		  HAL_Delay(100);
	  }


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
