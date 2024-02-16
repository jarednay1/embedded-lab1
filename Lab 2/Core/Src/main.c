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
int main(void) {
	HAL_Init(); // Reset of all peripherals, init the Flash and Systick
	SystemClock_Config(); //Configure the system clock
	
	// Enable AHBENR clock to enable GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Line 7848 of stm32f072xb.h has periph clocks
	
	// Enable ABP2ENR clock to enable SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	
	// Configure all LEDs: PC6, PC7, PC8, PC9, MODER register first
	GPIOC->MODER &= ~((1 << 19) | (1 << 17) | (1 << 15) | (1 << 13));
	GPIOC->MODER |= ((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12));
	
	// Configure OTYPER registers to be in push-pull mode
	GPIOC->OTYPER &= ~((1 << 9) | (1 << 8) | (1 << 7) | (1 << 6));
	
	// Configure the OSPEEDR registers to low speed mode
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12));
	
	// Configure the PUPDR registers to no pull up / down resistors
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18) | (1 << 17) | (1 << 16) | (1 << 15) | (1 << 14) | (1 << 13) | (1 << 12));
	
	// Set up the LED pins using the ODR register
	GPIOC->ODR |= (1 << 9);
	GPIOC->ODR &= ~((1 << 7) | (1 << 6));
	
	// Setup for USER push button located on pin PA0. Start with the MODER register
	// which will be set to input mode 0x00. Low speed mode on OSPEEDR register,
	// and pull down resistor on PUPDR register.
	
	GPIOA->MODER &= ~((1 << 1) | (1 << 0));
	GPIOA->OSPEEDR &= ~(1 << 0);
	GPIOA->PUPDR |= (1 << 1);
	GPIOA->PUPDR &= ~(1 << 0);
	
	// Setup for the EXTI, interupt control, register EXTI_IMR
	EXTI->IMR |= (1 << 0);
	
	// Setup for the EXTI_RTSR register, which will set input to trigger on rising edge
	EXTI->RTSR |= (1 << 0);
	
	// Set the SYSCFG_ERCI[0] regsiter to have PA[0] as an input.
	SYSCFG->EXTICR[0] &= ((1 << 2) | (1 << 1) | (1 << 0));
	
	// Enable NVIC EXTI line 0 
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(SysTick_IRQn, 2);
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	
	
	//static volatile uint32_t counter = 0;
	while (1) {
		HAL_Delay(500);
		GPIOC->ODR ^= (1 << 6);
	}
}	

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
