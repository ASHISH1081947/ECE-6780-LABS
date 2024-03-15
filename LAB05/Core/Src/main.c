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
	  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
 
  // Enable GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIO_InitTypeDef initc6789 = {GPIO_PIN_8 | GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
	 HAL_GPIO_Init(GPIOC, &initc6789);
	
	 // Configure PB11 in alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER11);  // Clear the bits
    GPIOB->MODER |= (GPIO_MODER_MODER11_1); // Set to alternate function mode

    // Configure PB11 as open-drain output
    GPIOB->OTYPER |= GPIO_OTYPER_OT_11;

    // Configure the alternate function for PB11 (assuming it's AF4 for I2C2_SDA, check your datasheet/reference manual)
    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11; // Clear the bits
    GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL11_Pos); // Set AF4 for I2C2_SDA

    // Configure PB13 in alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER13);  // Clear the bits
    GPIOB->MODER |= (GPIO_MODER_MODER13_1); // Set to alternate function mode

    // Configure PB11 as open-drain output
    GPIOB->OTYPER |= GPIO_OTYPER_OT_13;

    // Configure the alternate function for PB13 (assuming it's AF4 for I2C2_SDA, check your datasheet/reference manual)
    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL13; // Clear the bits
    GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFSEL13_Pos); // Set AF4 for I2C2_SCL
 
    // Configure PB14 in general purpose output mode
    GPIOB->MODER &= ~GPIO_MODER_MODER14;  // Clear the bits
    GPIOB->MODER |= GPIO_MODER_MODER14_0; // Set to general purpose output mode

    // Configure PB14 as push-pull output
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;

    // Initialize/set PB14 high
    GPIOB->BSRR |= GPIO_BSRR_BS_14;
		
		// Configure PC0 in general purpose output mode
    GPIOC->MODER &= ~GPIO_MODER_MODER0;  // Clear the bits
    GPIOC->MODER |= GPIO_MODER_MODER0_0; // Set to general purpose output mode

    // Configure PC0 as push-pull output
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;

    // Initialize/set PC0 high
    GPIOC->BSRR |= GPIO_BSRR_BS_0;

    // Enable I2C2
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	  // Set the parameters for 100 kHz standard-mode in TIMINGR register
    // Assuming a 16 MHz system clock, adjust the values accordingly
    I2C2->TIMINGR = (0x1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0X4 << 20);

    // Enable the I2C2 peripheral using the PE bit in the CR1 register
    I2C2->CR1 |= I2C_CR1_PE;
		
//		// Configure-PC6-and-PC7-as-output

//    GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0); // Output mode
//    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7); // Push-pull 
//	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7); // High speed 
//	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7); // No pull-up, no pull-down
//	
//  	// Configure-PC8-and-PC9-as-output

//    GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0); // Output mode
//    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9); // Push-pull 
//	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR8 | GPIO_OSPEEDR_OSPEEDR9); // High speed 
//	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9); // No pull-up, no pull-down
	
		
		
		// Step 1: Set transaction parameters in CR2 register
    I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_START;
     
    // Step 2: Wait until either TXIS or NACKF flags are set
    while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
      // Wait
    }

    // If NACKF flag is set, there might be an error
    if (I2C2->ISR & I2C_ISR_NACKF) {
     GPIOC->ODR ^= GPIO_ODR_6;   // Handle error (e.g., print error message or set LED pattern)   
    }

    // Step 3: Write the address of the "WHO_AM_I" register into TXDR
    I2C2->TXDR = 0x0F;

    // Step 4: Wait until TC flag is set
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        // Wait
    }

    // Step 5: Reload CR2 register with read operation parameters and set START bit
    I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;

    // Step 6: Wait until either RXNE or NACKF flags are set
    while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {
        // Wait
    }

    // If NACKF flag is set, there might be an error
    if (I2C2->ISR & I2C_ISR_NACKF) {
        GPIOC->ODR ^= GPIO_ODR_8; // Handle error (e.g., print error message or set LED pattern)
    }

    // Step 7: Wait until TC flag is set
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        // Wait
    }

    // Step 8: Check the contents of RXDR register
    if (I2C2->RXDR == 0xD3) {
        // WHO_AM_I register value matches the expected value
       GPIOC->ODR ^= GPIO_ODR_9;  // Green LED Glows
    } 
		else {
        // WHO_AM_I register value doesn't match the expected value
       GPIOC->ODR ^= GPIO_ODR_6;  // RED LED Glows
    }

    // Step 9: Set the STOP bit in CR2 register
    I2C2->CR2 |= I2C_CR2_STOP;
	
//    // Step 1: Enable GPIOB and GPIOC in the RCC
//    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable clock for GPIOB
//    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable clock for GPIOC
// GPIO_InitTypeDef initc6789 = {GPIO_PIN_8 | GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
//	 HAL_GPIO_Init(GPIOC, &initc6789);
//    // Step 2: Set PB11 to alternate function mode, open-drain output type, and select I2C2_SDA as its alternate function
//    GPIOB->MODER |= GPIO_MODER_MODER11_1; // Set PB11 to alternate function mode
//    GPIOB->OTYPER |= GPIO_OTYPER_OT_11;   // Set PB11 as open-drain output type
//    GPIOB->AFR[1] |= (1 << ((11 - 8) * 4)); // Select I2C2_SDA as alternate function for PB11

//    // Step 3: Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function
//    GPIOB->MODER |= GPIO_MODER_MODER13_1; // Set PB13 to alternate function mode
//    GPIOB->OTYPER |= GPIO_OTYPER_OT_13;   // Set PB13 as open-drain output type
//    GPIOB->AFR[1] |= (1 << ((13 - 8) * 4)); // Select I2C2_SCL as alternate function for PB13

//    // Step 4: Set PB14 to output mode, push-pull output type, and initialize/set the pin high
//    GPIOB->MODER |= GPIO_MODER_MODER14_0; // Set PB14 to output mode
//    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;  // Set PB14 as push-pull output type
//    GPIOB->BSRR |= GPIO_BSRR_BS_14;      // Set PB14 high

//    // Step 5: Set PC0 to output mode, push-pull output type, and initialize/set the pin high
//    GPIOC->MODER |= GPIO_MODER_MODER0_0; // Set PC0 to output mode
//    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;  // Set PC0 as push-pull output type
//    GPIOC->BSRR |= GPIO_BSRR_BS_0;      // Set PC0 high

//    // Step 6: Enable the I2C2 peripheral in the RCC
//    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; // Enable clock for I2C2

//    // Step 7: Set the parameters in the TIMINGR register to use 100 kHz standard-mode I2C
//    I2C2->TIMINGR = (uint32_t)0x00303D5B; // Example timing parameters for 100 kHz standard-mode I2C

//    // Step 8: Enable the I2C peripheral using the PE bit in the CR1 register
//    I2C2->CR1 |= I2C_CR1_PE; // Enable I2C peripheral
//    // Step 9: Wait until either TXIS or NACKF flags are set
//	
//    while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));

//    // Step 10: Write the address of the "WHO_AM_I" register into the TXDR
//    I2C2->TXDR = 0x0F; // Address of the "WHO_AM_I" register

//    // Step 11: Wait until the TC flag is set
//    while (!(I2C2->ISR & I2C_ISR_TC));

//    // Step 12: Reload the CR2 register with parameters for a read operation
//    I2C2->CR2 = (0x69 << 1) | (1 << 16) | (1 << 10) | I2C_CR2_START;

//    // Step 13: Wait until either RXNE or NACKF flags are set
//    while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));

//    // Step 14: Wait until the TC flag is set
//    while (!(I2C2->ISR & I2C_ISR_TC));

//    // Step 15: Check the contents of the RXDR register
//    //if ((I2C2->RXDR) == 0xD3) {
//			GPIOC->ODR ^= GPIO_ODR_7; // Toggle BLUE LED
//			
// 
//    //}

//    // Step 16: Set the STOP bit in the CR2 register
//    I2C2->CR2 |= I2C_CR2_STOP;

//    while (1) {
//        // Your main loop code goes here
//    }
}


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  /* USER CODE END 3 */


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
