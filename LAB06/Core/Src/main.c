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
	uint16_t potValue = 0;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	// Enable GPIOC clock
   RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// Enable ADC clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	 GPIO_InitTypeDef initc6789 = {GPIO_PIN_8 | GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
	 HAL_GPIO_Init(GPIOC, &initc6789);
   GPIOC->MODER |= GPIO_MODER_MODER0; // Set PC0 as analog mode
		//set adc 8 bit bit 3&4, continuos conversion mode bit13, hardware trigge disabled - 10,11
		ADC1->CFGR1 |= ((1<<4) | (1<<3)| (1<<13) | (0<<10) | (0<<11));
	
		ADC1->CHSELR |= (1<<10); //enable input channel 10 for pin 0
    ADC1->CR |= (0<<0); //clear aden
		ADC1->CFGR1 |= (0<<0); //clear dmaen
	//calibration
		if ((ADC1->CR & ADC_CR_ADEN) != 0) 
	{
		ADC1->CR |= ADC_CR_ADDIS; 
	}
		while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
	}
		ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; 
		ADC1->CR |= ADC_CR_ADCAL; 
	
	//enable adcen
		while ((ADC1->CR & ADC_CR_ADCAL) != 0) 
	{
		GPIOC->ODR |= (1<<6);
	}
		if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) 
		{
			ADC1->ISR |= ADC_ISR_ADRDY; 
		}
		ADC1->CR |= ADC_CR_ADEN;
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
		{
			GPIOC->ODR |= (1<<7);
		}
    
  while (1)
  {
		ADC1->CR |= ADC_CR_ADSTART; // Start conversion
    
    while (!(ADC1->ISR & ADC_ISR_EOC))
		{
		
			GPIOC->ODR |= (1<<6); 
		}; 
    potValue = ADC1->DR;
		if (potValue >= 10) 
			{
            GPIOC->ODR |= (1<<6) ; // Turn on RED LED 
      } 
		else 
			{
            GPIOC->ODR &= ~(1<<6); // Turn off RED LED 
      }
     if (potValue >= 30) 
			{
            GPIOC->ODR |= (1<<7); // Turn on Blue LED
      } 
			else 
			{
            GPIOC->ODR &= ~(1<<7); // Turn off Blue LED
      }
      if (potValue >= 40)
		  {
            GPIOC->ODR |= (1<<8); // Turn on Orange LED
      } 
			else 
			{
            GPIOC->ODR &= ~(1<<8); // Turn off Orange LED
      }
      if (potValue >= 20) 
			{
            GPIOC->ODR |= (1<<9); // Turn on Green LED
      } 
			else 
			{
            GPIOC->ODR &= ~(1<<9); // Turn off Green LED
      }
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
