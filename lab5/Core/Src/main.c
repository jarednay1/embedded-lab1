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
void GPIO_init(void);
void USART_init(void);
void I2C2_init(void);
void Gyro_init(void);
void Transmit_Char(char input);
void Transmit_String(char* input);
char Read_Slave(char slave_address, char register_address, char is_last_byte);
void Write_Slave(char slave_address, char data, char is_last_byte);
int16_t Read_X(void);
int16_t Read_Y(void);

// CONSTS
const char Gyro_Address = 0x69;
const char XL_Address = 0x28;
const char XH_Address = 0x29;
const char YL_Address = 0x2A;
const char YH_Address = 0x2B;
const int16_t threshold = 0x01FF;

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

// Helper to handle all GPIO setup
void GPIO_init(void) {
	// Set up clock for GIPIOB and GPIOC
	RCC->AHBENR |= (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN);
	
	// ----START I2C GPIO SETUP---- 
	// Pin PB11
	// Set to Alternate Function Mode
	GPIOB->MODER &= ~(1 << 22);
	GPIOB->MODER |= (1 << 23);
	
	// Set to open drain
	GPIOB->OTYPER |= (1 << 11);
	
	// Set alternate funtion to I2C2_SDA - AF1
	GPIOB->AFR[1] &= ~((1 << 15) | (1 << 14) | (1 << 13));
	GPIOB->AFR[1] |= (1 << 12);
	
	// Pin PB13
	// Set to Alternate Function Mode
	GPIOB->MODER &= ~(1 << 26);
	GPIOB->MODER |= (1 << 27);
	
	// Set to open drain
	GPIOB->OTYPER |= (1 << 13);
	
	// Set alternate function to ISC2_SCL - AF5
	GPIOB->AFR[1] &= ~((1 << 23) | (1 << 21));
	GPIOB->AFR[1] |= ((1 << 22) | (1 << 20));
	
	// Pin PB14
	// Set to output mode
	GPIOB->MODER &= ~(1 << 29);
	GPIOB->MODER |= (1 << 28);
	
	// Set to push-pull resistors
	GPIOB->OTYPER &= ~(1 << 14);
	
	// Set output state to high
	GPIOB->ODR |= (1 << 14);
	
	// Pin PC0
	// Set to output mode
	GPIOC->MODER &= ~(1 << 1);
	GPIOC->MODER |= (1 << 0);
	
	// Set to push-pull resistors
	GPIOC->OTYPER &= ~(1 << 0);
	
	// Set output state to high
	GPIOC->ODR |= (1 << 0);
}

// Helper for USART initilization PC4 = TX and PC5 = RX
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
	USART3->CR1 |= (1 << 5);
	
	// Enable USART interupt in the NVIC
	NVIC_EnableIRQ(USART3_4_IRQn);
}

// Helper to handle I2C setup
void I2C2_init(void) {
	// Enable the clock for I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Set timing register configurations for 100kHz (Standard mode)
	// Set prescaler value to 1
	I2C2->TIMINGR |= (1 << 28);
	
	// Set the SCL low period to 0x13
	I2C2->TIMINGR &= ~((1 << 7) | (1 << 6) | (1 << 5) | (1 << 3) | (1 << 2));
	I2C2->TIMINGR |= ((1 << 4) | (1 << 1) | (1 << 0));
	
	// Set the SCL high period to 0xF
	I2C2->TIMINGR &= ~((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12));
	I2C2->TIMINGR |= ((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
	
	// Set the data hold time to 0x2
	I2C2->TIMINGR &= ~((1 << 19) | (1 << 18) | (1 << 16));
	I2C2->TIMINGR |= (1 << 17);
	
	// Set up data setup time to 0x4
	I2C2->TIMINGR &= ~((1 << 23) | (1 << 21) | (1 << 20));
	I2C2->TIMINGR |= (1 << 22);
	
	// Enable in the control register
	I2C2->CR1 |= (1 << 0);
}

void Gyro_init(void) {

}

// Helper function for transmitting one character through USART
void Transmit_Char(char input) {
	// An empty while loop that breaks through when the status register
	// says the transmit data register is empty.
	while(!(USART3->ISR & (1 << 7))) {
			
	}
	
	// Write the input to the transfer data register
	USART3->TDR = input;
}

// Helper function to transmit a string through USART
void Transmit_String(char* input) {
	int counter = 0;
	
	// Loop through and transmit the string.
	while(*(input + counter) != 0) {
		Transmit_Char(*(input + counter));
		counter = counter + 1;
	}
}

// Helper method to read from slave device registers.
char Read_Slave(char slave_address, char register_address, char is_last_byte) {
	// Some locals to keep the return value
	char data;
	
	// Clear the register if there is a value in it, only bits [7:1] are of worry.
	//I2C2->CR2 &= ~(0xFF << 1);
	
	// Set the slave address shifted one to the left.
	I2C2->CR2 |= (slave_address << 1);
	
	// Clear the bytes register.
	//I2C2->CR2 &= ~((0xFF << 16));
	
	// Set number of bytes to transfer, this is selecting the regsister
	I2C2->CR2 |= (0x1 << 16);
	
	// Set to read operation.
	I2C2->CR2 |= (1 << 10);
	
	// Set the start bit.
	I2C2->CR2 |= (1 << 13);
	

	// While waiting for RXNE or NACKF to be set.
	while (!(I2C2->ISR & (1 << 2) | I2C2->ISR & (1 << 4))) {
			
	}
	
	// If the NACK flag is there print error message.
	if (I2C2->ISR & (1 << 4)) {
		Transmit_String("error read");
		return 0x00;
	}
	
	// Get the value from transmission.
	data = I2C2->RXDR;
	
	// Wait for transfer complete flag to be set.
	while (!(I2C2->ISR & (1 << 6))) {
		
	}
	
	// Set the stop bit and return a ptr to the data.
	if (is_last_byte) {
		I2C2->CR2 |= (1 << 14);
	}
	return data;
}


// Helper function to write to a slave device at given register
void Write_Slave(char slave_address, char data, char is_last_byte) {
	// Clear the register if there is a value in it, only bits [7:1] are of worry.
	//I2C2->CR2 &= ~(0xFF << 1);
	
	// Set the slave address shifted one to the left.
	I2C2->CR2 |= (slave_address << 1);
	
	// Clear the bytes register.
	//I2C2->CR2 &= ~((0xFF << 16));
	
	// Set number of bytes to transfer
	I2C2->CR2 |= (0x1 << 16);
	
	// Set to write operation.
	I2C2->CR2 &= ~(1 << 10);
	
	// Set the start bit.
	I2C2->CR2 |= (1 << 13);
	
	// While waiting for TXIS or NACKF to be set.
	while (!(I2C2->ISR & (1 << 1) | I2C2->ISR & (1 << 4))) {
		
	}
	
	//Transmit_String("Made it past TXIS");
	// If the NACK flag is there print error message.
	if (I2C2->ISR & (1 << 4)) {
		Transmit_String("error write");
		return;
	}
	
	// Set the address of the register on target chip.
	I2C2->TXDR = data;
	
	// Wait for transfer complete flag to be set
	while (!(I2C2->ISR & (1 << 6))) {
		
	}
	
	// Set the stop bit
	if (is_last_byte) {
		I2C2->CR2 |= (1 << 14);
	}
}

// Heleper method to get the value from gyro of X
int16_t Read_X(void) {
	int16_t return_val;
	char temp;
	
	//GPIOC->ODR |= (1 << 9);
	Write_Slave(Gyro_Address, XH_Address, 0);
	//GPIOC->ODR |= (1 << 7);
	temp = Read_Slave(Gyro_Address, XH_Address, 0);
	return_val = (temp << 8);
	

	
	Write_Slave(Gyro_Address, XL_Address, 0);
	temp = Read_Slave(Gyro_Address, XL_Address, 0);
	return_val |= (return_val | temp);
	
	return return_val;
}

int16_t Read_Y(void) {
	return 0;
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
	GPIO_init();
	I2C2_init();
	USART_init();
	
	GPIOC->ODR &= ~((1 << 9) | (1 << 8) | (1 << 7) | (1 << 6));
	
	char x_bytes[2];
	char y_bytes[2];
	short* x;
	short* y;
	// Gyro setup
	Write_Slave(Gyro_Address, 0x20, 0);
	Write_Slave(Gyro_Address, 0x0B, 0);
	
	//I2C2->CR2 |= (1 << 14);
  while (1) {
		
		Transmit_Char('F');
		// For part 1
		/*
		Write_Slave(Gyro_Address, 0x0F, 0);
		char whoami = Read_Slave(Gyro_Address, 0x0F, 1);
		if (whoami == 0) {
			Transmit_String("Bad return value");
			break;
		}
		whoami = whoami - 0x92;
		Transmit_Char(whoami);
		*/
		

		
		Write_Slave(Gyro_Address, 0x20, 0);
		char ctrl_reg = Read_Slave(Gyro_Address, 0x20, 1);
		
		ctrl_reg = ctrl_reg + 0x41;
		
		Transmit_Char(ctrl_reg);
		
		
		
		int16_t x_val = Read_X();
		
		if (x_val > threshold) {
			GPIOC->ODR |= (1 << 6);
		}
		else {
			GPIOC->ODR &= ~(1 << 6);
		}
		
		if (x_val < 0 - threshold) {
			GPIOC->ODR |= (1 << 8);
		}			
		else {
			GPIOC->ODR &= ~(1 << 8);
		}
		
		HAL_Delay(100);

		
		/*
		// Read out the data and store into shorts
		x_bytes[0] = Read_Slave(Gyro_Address, 0x29);
		x_bytes[1] = Read_Slave(Gyro_Address, 0x28);
		y_bytes[0] = Read_Slave(Gyro_Address, 0x2B);
		y_bytes[1] = Read_Slave(Gyro_Address, 0x2A);
		x_bytes[0] = x_bytes[0] + 0x42;
		Transmit_Char(x_bytes[0]);
		
		*x = (x_bytes[0] << 8) | x_bytes[1];
		*y = (y_bytes[0] << 8) | y_bytes[1];
		
		if (*x > 0){
			GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~(1 << 9);
		}
		else if (*x <= 0) {
			GPIOC->ODR |= (1 << 9);
			GPIOC->ODR &= ~(1 << 6);
		}
		*/
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
