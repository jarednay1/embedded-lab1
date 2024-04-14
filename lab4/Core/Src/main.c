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
void LED_init(void);
void USART_init(void);
void Transmit_Char(char input);
void Transmit_String(char* input);
void Recieve_Char(char input);

// Helper for LED initialization
void LED_init(void) {
	// First enable clock for GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set up MODER registers to general purpose output for PC6, PC7,
	// PC8, and PC9, the LEDs
	GPIOC->MODER |= ((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12));
	GPIOC->MODER &= ~((1 << 19) | (1 << 17) | (1 << 15) | (1 << 13));
	
	// Set up OTYPER registers to be in push-pull mode
	GPIOC->OTYPER &= ~((1 << 9) | (1 << 8) | (1 << 7) | (1 << 6));
	
	// Set up OSPEEDR registers to low speed mode
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12));
	
	// Set up PUPDR registers to no pull up / no pull down resistors
	GPIOC->PUPDR &= ~((1 << 19) | (1 << 18) | (1 << 17) | (1 << 16));
	GPIOC->PUPDR &= ~((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12));
}

// Helper for USART initilization
void USART_init(void) {
	// First enable clock for GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set both PC4 and PC5 to Alternate Function Mode 
	GPIOC->MODER |= ((1 << 11) | (1 << 9));
	GPIOC->MODER &= ~((1 << 10) | (1 << 8));
	
	// Select the AF1 alternate function for PC4 and PC5
	GPIOC->AFR[0] |= ((1 << 20) | (1 << 16));
	
	// Enable clock for USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set the baud rate to 115200 bits / second. Clock is 8Mhz.
	USART3->BRR = 8000000 / 115200;
	
	// Enable Transmitter and Reciever hardware
	USART3->CR1 |= ((1 << 3) | (1 << 2));
	
	// Enable the USART
	USART3->CR1 |= (1 << 0);
	
	// Enable interupts
	USART3->CR1 |= (1 << 7);
	
	// Enable USART interupt in the NVIC
	//NVIC_EnableIRQ(USART3_4_IRQn);
}

// Function for transmitting one character
void Transmit_Char(char input) {
	// An empty while loop that breaks through when the status register
	// says the transmit data register is empty.
	while(!(USART3->ISR & (1 << 7))) {
			
	}
	
	// Write the input to the transfer data register
	USART3->TDR = input;
}

// Function to transmit a string
void Transmit_String(char* input) {
	int counter = 0;
	
	// Loop through and transmit the string.
	while(*(input + counter) != 0) {
		Transmit_Char(*(input + counter));
		counter = counter + 1;
	}
}

// Method to recieve input for toggling LEDs
void Recieve_Char(char input) {
	//char input = USART3->RDR;
	char* error_msg = "Incorrect Key";
	
	if (input == 'g') {
		GPIOC->ODR ^= (1 << 9);
	} 
	else if (input == 'o') {
		GPIOC->ODR ^= (1 << 8);
	}
	else if (input == 'b') {
		GPIOC->ODR ^= (1 << 7);
	}
	else if (input == 'r') {
		GPIOC->ODR ^= (1 << 6);
	}
	else {
		Transmit_String(error_msg);
	}
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

	LED_init();
	USART_init();

  /* Infinite loop */
  while (1) {
		char* test_input = "hello world";
		HAL_Delay(1000);
		//Transmit_Char('a');
		//Transmit_String(test_input);
		
		if (USART3->ISR & (1 << 5)) {
			char input = USART3->RDR;
			Recieve_Char(input);
		}
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
