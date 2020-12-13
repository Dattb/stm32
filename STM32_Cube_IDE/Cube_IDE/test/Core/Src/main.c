/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
 uint8_t Mode_Color;
 char c;
 uint16_t adc_value=32456;

 unsigned char vruc_Cycle;
 unsigned char vruc_Count;
 unsigned char BUTTON_MAX_DEBOUCE=5;
 uint8_t _b_cnt;
 unsigned short adc_v1,adc_v2,adc_v3,adc_v4,adc_v5;
 unsigned short _button_press_b1;
 unsigned short _button_press_b2;
 unsigned short _button_press_b3;
 unsigned short _button_press_b4;
 unsigned short _button_press_b5;
 uint8_t BUTTON[5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void BUTTON_DEBOUCE();
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
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
 	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
 	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
 	HAL_ADC_Start(&hadc1);
 	unsigned short duty1,duty2,duty3;
 	//unsigned short trang_thai_truoc, trang_thai_sau,button_count,button_state;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {   // chong rung
	  if(!BUTTON_DEBOUCE()) Mode_Color++;
	  while(!BUTTON_DEBOUCE());
	  //else HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 0);
//	  // viet ham print de in ra ca3 gia tri
//	   HAL_ADC_ConvCpltCallback(&hadc1);
//	   adc_v1=adc_value/10000+48;
//	   HAL_UART_Transmit(&huart4,&adc_v1,1,1000);
//       adc_v2=((adc_value-(adc_v1-48)*10000)/1000)+48;
//       HAL_UART_Transmit(&huart4,&adc_v2,1,1000);
//       adc_v3=(adc_value-(adc_v1-48)*10000-(adc_v2-48)*1000)/100+48;
//       HAL_UART_Transmit(&huart4,&adc_v3,1,1000);
//		adc_v4=(adc_value-(adc_v1-48)*10000-(adc_v2-48)*1000-(adc_v3-48)*100)/10+48;
//		HAL_UART_Transmit(&huart4,&adc_v4,1,1000);
//		adc_v5=(adc_value-(adc_v1-48)*10000-(adc_v2-48)*1000-(adc_v3-48)*100-(adc_v4-48)*10)+48;
//		HAL_UART_Transmit(&huart4,&adc_v5,1,1000);
//		HAL_Delay(100);
//       char str[]="\r\n"; HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);

	  if( HAL_UART_Receive(&huart4, &c,1, 0)==HAL_OK)
	  	 {  switch (c)
	  	   {

	           	 case  'a' : {Mode_Color=1; char str[]="Color1\r\n"; HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);break;}
	           	 case  'b' : {Mode_Color=2; char str[]="Color2\r\n"; HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);break;}
	           	 case  'c' : {Mode_Color=3; char str[]="Color3\r\n"; HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);break;}
	           	 case  'd' : {Mode_Color=4; char str[]="Color4\r\n"; HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);break;}
	           	 case  'e' : {Mode_Color=5; char str[]="Color5\r\n"; HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);break;}
	           	 case  'f' : {Mode_Color=6; vruc_Cycle = 0; vruc_Count = 0; char str[]="Color6\r\n"; HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);break;}
	  	   }

	  	 }
	  	switch(Mode_Color)
	  	{


	  		case 1:
	  		   {
	  				bam_xung (1,99,99 );
	  				char str[]="Color1\r\n";
	  				//HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);
	  				//HAL_Delay(100);
	  				break;
	  		   }
	  		case 2:
	  		   {
	  				bam_xung (99,1,99 );
	  				char str[]="Color2\r\n";
	  				//HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);
	  				//HAL_Delay(100);
	  				break;
	  		   }
	  		case 3:
	  		   {
	  				bam_xung (99,99,1);
	  				char str[]="Color3\r\n";
	  				//HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);
	  				//HAL_Delay(100);
	  				break;
	  		   }
	  		case 4:
	  		   {
	  				bam_xung (1,1,99);
	  				char str[]="Color4\r\n";
	  				//HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);
	  				//HAL_Delay(100);
	  				break;
	  		   }
	  		case 5:
	  		   {
	  				bam_xung (99,1,1);
	  				char str[]="Color5\r\n";
	  				//HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);
	  				//HAL_Delay(100);
	  				break;
	  		    }
	  		case 6:
	  		   {
	  			   vruc_Count++;
	  			   if(vruc_Count > 99){
	  				   vruc_Count = 0;
	  				   vruc_Cycle++;
	  				   if(vruc_Cycle > 5)
	  				   {
	  					   vruc_Cycle = 0;
	  				   }
	  			   char str[]="Change Mode\r\n";
	  			   HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);

	  			   }
	  //			   char str[]="Change Mode\r\n";
	  //			   HAL_UART_Transmit(&huart4,(uint8_t *)&str,sizeof(str),1000);
	  			   switch(vruc_Cycle){
	  				case 0:
	  					bam_xung (99,vruc_Count,1);
	  					break;
	  				case 1:
	  					bam_xung (99-vruc_Count,99,1);
	  					break;
	  				case 2:
	  					bam_xung (1,99,vruc_Count);
	  					break;
	  				case 3:
	  					bam_xung (1,99-vruc_Count,99);
	  					break;
	  				case 4:
	  					bam_xung (vruc_Count,1,99);
	  					break;
	  				case 5:
	  					bam_xung (99,1,99-vruc_Count);
	  					break;
	  				default:
	  					break;
	  			   }

	  			   break;
	  		   }
	  		default:
	  			Mode_Color=1;break;
	  	}
	  	HAL_Delay(20);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 499;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 PD14 
                           PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bam_xung (int duty1,int duty2, int duty3 )
  {
	    //duty1 =pwm1;duty2=pwm2;duty3=pwm3;
	    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10*duty1);
	    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,10*duty2);
	    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,10*duty3);

  }
void BUTTON_DEBOUCE()
{

		if (!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)) {
			_b_cnt++;
			if (_b_cnt > (128 + BUTTON_MAX_DEBOUCE)){
				_b_cnt = (128 + BUTTON_MAX_DEBOUCE);
				_button_press_b1 = 1;
			}
		}
		else {
			_b_cnt--;
			if (_b_cnt < (128 - BUTTON_MAX_DEBOUCE)){
				_b_cnt = (128 - BUTTON_MAX_DEBOUCE);
				_button_press_b1 = 0;
			}
		}

  return _button_press_b1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance==hadc1.Instance) adc_value = HAL_ADC_GetValue(&hadc1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
