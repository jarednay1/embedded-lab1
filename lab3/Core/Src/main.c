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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LED_Setup(void);
void Timer_Setup(void);

void LED_Setup(void) {
	// -----START LED SETUP-----
	// Enable AHBENR clock to enable GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// First Orange and Green LEDs
	// Enable MODER register in orange and green LEDs GP output mode
	GPIOC->MODER &= ~((1 << 19) | (1 << 17));
	GPIOC->MODER |= ((1 << 18) | (1 << 16));
	
	// Enable OTYPER registers to be in push-pull mode
	GPIOC->OTYPER &= ~((1 << 9) | (1 << 8));
	
	// Enable OSPEEDR registers to low speed mode
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 16));
	
	// Enable PUPDR registers to no pull up or pull down resistors
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18) | (1 << 17) | (1 << 16));
	
	// Set up the LED pins to on / off in the ODR register
	GPIOC->ODR &= ~(1 << 9);
	GPIOC->ODR |= (1 << 8);
	
	// Next setup for the Blue and Red LEDs
	// Set both to Alternate Function Mode
	GPIOC->MODER |= ((1 << 15) | (1 << 13));
	GPIOC->MODER &= ~((1 << 14) | (1 << 12));
	
	// Set AFR registers
	GPIOC->AFR[0] &= ~((1 << 31) | (1 << 30) | (1 << 29) | (1 << 28) | (1 << 27) | (1 << 26) | (1 << 25) | (1 << 24));
	
}

void Timer_Setup(void) {
	// -----START TIMER SETUP-----
	// Code for 3.2
	// Enable RCC clock for timer2 and timer3
	RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN);
	
	// Enable PSC register to 39 or 0x27
	//TIM3->PSC = 0x27;
	TIM3->PSC = 99;
	
	// Enable ARR register to 125 or 0x7D
	//TIM3->ARR = 0x7D;
	TIM3->ARR = 100;
	
	// Set CCMR1 register to have both CC1S and CC2S set to output mode
	TIM3->CCMR1 &= ~(/*(1 << 12) |*/ (1 << 9) | (1 << 8) | (1 << 1) | (1 << 0));
	TIM3->CCMR1 |= ((1 << 14) | (1 << 13) | (1 << 12) | (1 << 11) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3));
	
	// Set output enable bits for channels 1 and 2 in CCER register
	TIM3->CCER |= ((1 << 4) | (1<< 0));
	
	// Set CCR1 and CCR2 to 20% of AAR which is 25 or 0x19
	TIM3->CCR1 = 85;
	TIM3->CCR2 = 1;
	//TIM3->CCR1 = 10;
	//TIM3->CCR2 = 10;
	
	// Disable control register for TIM3
	TIM3->CR1 |= (1 << 0);
	
	// Code for 3.1
	// Enable RCC clock for timer2 and timer3
	//RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN);
	
	// Enable PSC register to 7999 or 0x1F3F
	TIM2->PSC = 0x1F3F;
	
	// Enable ARR register to 4000 or 0xFA0
	TIM2->ARR = 0xFA0;
	
	// Enable DIER register to UIE(Update interrupt enable)
	TIM2->DIER |= 1;
	
	// Enable the CNT or configuration register of the timer
	TIM2->CR1 &= ~((1 << 9) | (1 << 8) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1	<< 1));
	TIM2->CR1 |= ((1 << 7) | (1 << 2) | (1 << 0));
	
	// Enable timer2 in the NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
}



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
	
	// Set up LEDs and Timers
	LED_Setup();
	Timer_Setup();

	// Main while loop
  while (1)
  {
	
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
