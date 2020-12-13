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
#include "stdio.h"
#include "5110.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Delay_us(uint16_t time_us);								//Delay us using Timer1. (max 65535us)
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);	// Configure GPIO_OUTPUT
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);		// Configure GPIO_INPUT
void DHT_Start();												// Send a signal respons DHT11 sensor Start
uint8_t DHT_Check_Response();									// Check Response of DHT. return '1' if availible
uint8_t DHT_Read_Data();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
 unsigned short Mode_menu=1,Mode_giamsat=1,Mode_cambien=1;
 char c;
 unsigned char vruc_Cycle;
 unsigned char vruc_Count;
 unsigned char T=40,H=100;
 uint8_t int_RH, dec_RH, int_T, dec_T, Check_Sum, Respones_State;
 uint8_t d1,d2,d3;
 uint8_t Mode_Color,Mode_LCD=1;
 uint16_t delaymain=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_Base_Start(&htim4);

// khoi tao cam bien DHT11
		DHT_Start();
		Respones_State = DHT_Check_Response();
		int_RH = DHT_Read_Data();
		dec_RH = DHT_Read_Data();
		int_T  = DHT_Read_Data();
		dec_T	 = DHT_Read_Data();
		Check_Sum = DHT_Read_Data();
// khoi tao LCD
		LCD5110_setup();
		PCD8544_backlight_state(ON);
		HAL_Delay(2000);
		PCD8544_backlight_state(OFF);
		HAL_Delay(2000);
		PCD8544_clear_screen(WHITE);
		PCD8544_backlight_state(ON);
		HAL_Delay(3000);
		PCD8544_clear_screen(WHITE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		if(!HAL_GPIO_ReadPin(btn_Mode_Color_GPIO_Port, btn_Mode_Color_Pin))
//		{
//		while(!HAL_GPIO_ReadPin(btn_Mode_Color_GPIO_Port, btn_Mode_Color_Pin)); Mode_Color++;
//		}
//		if(!HAL_GPIO_ReadPin(btn_Mode_LCD_GPIO_Port,btn_Mode_LCD_Pin))
//		{
//		while(!HAL_GPIO_ReadPin(btn_Mode_LCD_GPIO_Port,btn_Mode_LCD_Pin)); Mode_LCD++;
//		}

	  // doc DHT11



		if( HAL_UART_Receive(&huart3, &c,1, 0)==HAL_OK)
		 {  switch (c)
		   {

				 case  'a' : {Mode_Color=1; printf("color1\r\n");break;}
				 case  'b' : {Mode_Color=2; printf("color2\r\n");break;}
				 case  'c' : {Mode_Color=3; printf("color3\r\n");break;}
				 case  'd' : {Mode_Color=4; printf("color4\r\n");break;}
				 case  'e' : {Mode_Color=5; printf("color5\r\n");break;}
				 case  'f' : {Mode_Color=6; vruc_Cycle = 0; vruc_Count = 0;printf("color6\r\n"); break;}
				 case  'g' : {Mode_LCD=1;printf("Mode Menu 1\r\n");break;}
				 case  'h' : {Mode_LCD=2;printf("Mode Menu 2\r\n");break;}
				 case  'i' : {Mode_LCD=3;printf("Mode Menu 3\r\n");break;}
				 case  'k' : {Mode_LCD=4;printf("Mode Menu 4\r\n");break;}
		   }

		 }
		switch(Mode_Color)
		{


			case 1:
			   {
					bam_xung (1,99,99 );
					break;
			   }
			case 2:
			   {
					bam_xung (99,1,99 );
					break;
			   }
			case 3:
			   {
					bam_xung (99,99,1);
					break;
			   }
			case 4:
			   {
					bam_xung (1,1,99);
					break;
			   }
			case 5:
			   {
					bam_xung (99,1,1);
					break;
				}
			case 6:
			   {
				   vruc_Count++;
				   if(vruc_Count > 99)
				   {
					   vruc_Count = 0;
					   vruc_Cycle++;

					   if(vruc_Cycle > 5)
					   {
						   vruc_Cycle = 0;

					   }

				   }

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
		switch(Mode_LCD)
		 {
		  case 1: {
					//che do hien thi MAIN MENU
					PCD8544_clear_screen(WHITE);
					PCD8544_print_string(1, 0, "    MAIN MENU   ", BLACK);
					PCD8544_print_string(1, 2, "Cam Bien", WHITE);
					PCD8544_print_string(1, 3, "Giam Sat", WHITE);
					PCD8544_print_string(1, 4, "R G B", WHITE);

					if(!HAL_GPIO_ReadPin(btn_Xuong_GPIO_Port,btn_Xuong_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_Xuong_GPIO_Port,btn_Xuong_Pin));
						Mode_menu++;
						if(Mode_menu>3) Mode_menu=3;
					}
					if(!HAL_GPIO_ReadPin(btn_Len_GPIO_Port,btn_Len_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_Len_GPIO_Port,btn_Len_Pin));
						Mode_menu--;
						if(Mode_menu<1) Mode_menu=1;
					}
					switch(Mode_menu)
					   {
						case 1:
						{
							PCD8544_print_string(1, 2, "Cam Bien",BLACK);
							PCD8544_print_string(1, 3, "Giam Sat", WHITE);
							PCD8544_print_string(1, 4, "R G B", WHITE);
							if(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin))
							{
								while(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin));
								Mode_LCD=2;
							}
							break;
						}
						case 2:
						{
							PCD8544_print_string(1, 2, "Cam Bien", WHITE);
							PCD8544_print_string(1, 3, "Giam Sat", BLACK);
							PCD8544_print_string(1, 4, "R G B", WHITE);
							if(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin))
							{
								while(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin));
								Mode_LCD=3;
							}
							break;
						}
						case 3:
						{
							PCD8544_print_string(1, 2, "Cam Bien", WHITE);
							PCD8544_print_string(1, 3, "Giam Sat", WHITE);
							PCD8544_print_string(1, 4, "R G B", BLACK);
							if(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin))
							{
								while(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin));
								Mode_LCD=4;
							}
							break;
						}
						default: Mode_menu=1;break;
					}
					break;
				  }
		  case 2: {
					//che do hien thi nhiet do va do am len LCD
					PCD8544_clear_screen(WHITE);
					PCD8544_print_string(1, 0, "    Cam Bien    ", BLACK);
					PCD8544_print_string(1, 2, "Nhiet Do:", WHITE);print_int(45, 2,int_T, WHITE); PCD8544_set_cursor(70,2);PCD8544_print_char('C',WHITE);
					PCD8544_print_string(1, 3, "Do Am:", WHITE);print_int(45, 3,int_RH, WHITE); PCD8544_set_cursor(70,3);PCD8544_print_char('%',WHITE);
					if(!HAL_GPIO_ReadPin(btn_phai_GPIO_Port, btn_phai_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_phai_GPIO_Port, btn_phai_Pin));
                        Mode_cambien=1;
					}
					if(!HAL_GPIO_ReadPin(btn_Trai_GPIO_Port, btn_Trai_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_Trai_GPIO_Port, btn_Trai_Pin));
						Mode_cambien=2;
					}
					switch(Mode_cambien)
					{
					 case 1:
					 {int F= (9*int_T)/5+32;
					   PCD8544_print_string(1, 2, "Nhiet Do:", WHITE);print_int(45, 2,F, WHITE); PCD8544_set_cursor(70,2);PCD8544_print_char('F',WHITE);
					   PCD8544_print_string(1, 3, "Do Am:", WHITE);print_int(45, 3,int_RH, WHITE); PCD8544_set_cursor(70,3);PCD8544_print_char('%',WHITE);
					   break;
					 }
					 case 2:
					 {
					   PCD8544_print_string(1, 2, "Nhiet Do:", WHITE);print_int(45, 2,int_T, WHITE); PCD8544_set_cursor(70,2);PCD8544_print_char('C',WHITE);
					   PCD8544_print_string(1, 3, "Do Am:", WHITE);print_int(45, 3,int_RH, WHITE); PCD8544_set_cursor(70,3);PCD8544_print_char('%',WHITE);
					   break;
					 }
					}
					if(int_T>=T&&int_RH<H)
					{
					 PCD8544_print_string(1, 4, "     OVER T!    ", BLACK);	printf("OVER TEMPT!\r\n");
					}
					else if(int_RH>=H&&int_T<T)
					{
						PCD8544_print_string(1, 4, "     OVER H!    ", BLACK); printf("OVER HUM!\r\n");
					}
					else if(int_T>=T&&int_RH>=H)
					{
						PCD8544_print_string(1, 4, "    OVER T&H!   ", BLACK); printf("OVER HUM!\r\n");
					}
					else PCD8544_print_string(1, 4, "     SAVE    ", WHITE);
					if(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin));
						Mode_LCD=1;
					}
					break;
				  }
		  case 3: {
			        //che do giam sat nhiet do va do am len LCD
					PCD8544_clear_screen(WHITE);
					PCD8544_print_string(1, 0, "    Giam Sat    ", BLACK);
					PCD8544_print_string(1, 2, "Over T:", WHITE);print_int(45, 2,T, WHITE); PCD8544_set_cursor(70,2);PCD8544_print_char('C',WHITE);
					PCD8544_print_string(1, 3, "Over H:", WHITE);print_int(45, 3,H, WHITE); PCD8544_set_cursor(70,3);PCD8544_print_char('%',WHITE);

			        if(!HAL_GPIO_ReadPin(btn_Xuong_GPIO_Port,btn_Xuong_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_Xuong_GPIO_Port,btn_Xuong_Pin));
						Mode_giamsat++;
						if(Mode_giamsat>2) Mode_giamsat=2;
					}
					if(!HAL_GPIO_ReadPin(btn_Len_GPIO_Port,btn_Len_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_Len_GPIO_Port,btn_Len_Pin));
						Mode_giamsat--;
						if(Mode_giamsat<1) Mode_giamsat=1;
					}
					switch(Mode_giamsat)
					{
					 case 1:
					 {
						PCD8544_print_string(1, 2, "Over T:", BLACK);print_int(45, 2,T, WHITE); PCD8544_set_cursor(70,2);PCD8544_print_char('C',WHITE);
						PCD8544_print_string(1, 3, "Over H:", WHITE);print_int(45, 3,H, WHITE); PCD8544_set_cursor(70,3);PCD8544_print_char('%',WHITE);
						if(!HAL_GPIO_ReadPin(btn_phai_GPIO_Port, btn_phai_Pin))
						{
						  while(!HAL_GPIO_ReadPin(btn_phai_GPIO_Port, btn_phai_Pin));
						  T++;
						  if(T>50) T=50;
						}
						if(!HAL_GPIO_ReadPin(btn_Trai_GPIO_Port, btn_Trai_Pin))
						{
						  while(!HAL_GPIO_ReadPin(btn_Trai_GPIO_Port, btn_Trai_Pin));
						  T--;
						  if(T<0) T=0;
						}
						break;
					 }
					 case 2:
					 {
						PCD8544_print_string(1, 2, "Over T:",WHITE );print_int(45, 2,T, WHITE); PCD8544_set_cursor(70,2);PCD8544_print_char('C',WHITE);
						PCD8544_print_string(1, 3, "Over H:", BLACK);print_int(45, 3,H, WHITE); PCD8544_set_cursor(70,3);PCD8544_print_char('%',WHITE);
						if(!HAL_GPIO_ReadPin(btn_phai_GPIO_Port, btn_phai_Pin))
						{
						  while(!HAL_GPIO_ReadPin(btn_phai_GPIO_Port, btn_phai_Pin));
						  H++;
						  if(H>100) H=100;
						}
						if(!HAL_GPIO_ReadPin(btn_Trai_GPIO_Port, btn_Trai_Pin))
						{
						  while(!HAL_GPIO_ReadPin(btn_Trai_GPIO_Port, btn_Trai_Pin));
						  H--;
						  if(H<20) H=20;
						}
						break;
					 }
					}

					if(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin));
						Mode_LCD=1;
					}
					break;
				  }
		  case 4: {
					//hien thi che do RGB len LCD
					PCD8544_clear_screen(WHITE);
					if(!HAL_GPIO_ReadPin(btn_Len_GPIO_Port, btn_Len_Pin))
					{
					  while(!HAL_GPIO_ReadPin(btn_Len_GPIO_Port, btn_Len_Pin));
					  Mode_Color++;
					}
					if(!HAL_GPIO_ReadPin(btn_Xuong_GPIO_Port, btn_Xuong_Pin))
					{
					  while(!HAL_GPIO_ReadPin(btn_Xuong_GPIO_Port, btn_Xuong_Pin));
					  Mode_Color--;
					}
					PCD8544_print_string(1, 0, "      R G B     ", BLACK);
					PCD8544_print_string(1, 1, "Color:", WHITE);print_int(45, 1,Mode_Color, WHITE);
					PCD8544_print_string(1, 2, "% R:", WHITE);print_int(45, 2,d1, WHITE);
					PCD8544_print_string(1, 3, "% G:", WHITE);print_int(45, 3,d2, WHITE);
					PCD8544_print_string(1, 4, "% B:", WHITE);print_int(45, 4,d3, WHITE);
					if(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin))
					{
						while(!HAL_GPIO_ReadPin(btn_OK_GPIO_Port, btn_OK_Pin));
						Mode_LCD=1;
					}
					break;
				  }
		 }
		if(delaymain>20)
		{
			DHT_Start();
			Respones_State = DHT_Check_Response();
			int_RH = DHT_Read_Data();
			dec_RH = DHT_Read_Data();
			int_T  = DHT_Read_Data();
			dec_T  = DHT_Read_Data();
			Check_Sum = DHT_Read_Data();
			printf("nhiet do: %d\r\n", int_T);
			printf("Do am: %d\r\n", int_RH);
			//HAL_Delay(2000);

		 delaymain=0;
		}
		HAL_Delay(50);
		delaymain++;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 72-1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|DC_Pin|LIGHT_Pin|CS_Pin 
                          |SCLK_Pin|MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin DC_Pin LIGHT_Pin CS_Pin 
                           SCLK_Pin MOSI_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DC_Pin|LIGHT_Pin|CS_Pin 
                          |SCLK_Pin|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : btn_Len_Pin btn_OK_Pin btn_Xuong_Pin btn_Trai_Pin 
                           btn_phai_Pin */
  GPIO_InitStruct.Pin = btn_Len_Pin|btn_OK_Pin|btn_Xuong_Pin|btn_Trai_Pin 
                          |btn_phai_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bam_xung (int duty1,int duty2, int duty3 )
{
	d1=duty1;
	d2=duty2;
	d3=duty3;
	//duty1 =pwm1;duty2=pwm2;duty3=pwm3;
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,10*duty1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,10*duty2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,10*duty3);
}

void Delay_us(uint16_t time_us)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while(__HAL_TIM_GET_COUNTER(&htim4) <= time_us)
	{

	}
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT_Start()
{
	Set_Pin_Output(DHT11_GPIO_Port, DHT11_Pin);

	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
	Delay_us(18000);	// delay 18ms

	Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);

}

uint8_t DHT_Check_Response()
{
	uint8_t response=0;
	// wait for 40 us because length's signal is LOW 80us
	Delay_us(40);
	if(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))       // if LOW is ok
	{
		// wait for 40 us because length's signal is LOW 80us
		Delay_us(80);
		if(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
		{
			response =1;
		}
	}
	// waint Data_Pin pull LOW
	while(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
	{

	}
	return response;
}

uint8_t DHT_Read_Data()
{
	uint8_t data=0x00;
	for(int i=0; i<8; i++)
	{
		// wait Data_Pin Set LOW
		while(!HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
		{

		}
		Delay_us(40); // wait 40us between 28us and 70 us
		// if bit == 1
		if(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
		{
			data |= (1<<(7-i));
		}
		else
		{

		}
		while ((HAL_GPIO_ReadPin (DHT11_GPIO_Port, DHT11_Pin)));  // wait for the pin to go low
	}
	return data;
}



#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
 return ch;
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
