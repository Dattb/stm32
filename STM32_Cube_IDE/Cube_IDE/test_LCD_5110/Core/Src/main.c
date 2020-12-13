/* USER CODE BEGIN Header */
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
static const unsigned char font[][5] =
{
     {0x00, 0x00, 0x00, 0x00, 0x00} // 20
    ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
    ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
    ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
    ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
    ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
    ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
    ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
    ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
    ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
    ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
    ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
    ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
    ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
    ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
    ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
    ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
    ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
    ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
    ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
    ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
    ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
    ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
    ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
    ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
    ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
    ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
    ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
    ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
    ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
    ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
    ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
    ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
    ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
    ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
    ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
    ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
    ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
    ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
    ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
    ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
    ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
    ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
    ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
    ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
    ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
    ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
    ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
    ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
    ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
    ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
    ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
    ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
    ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
    ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
    ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
    ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
    ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
    ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
    ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
    ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ?
    ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
    ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
    ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
    ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
    ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
    ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
    ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
    ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
    ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
    ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
    ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
    ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
    ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
    ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
    ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
    ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
    ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
    ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
    ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
    ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
    ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
    ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
    ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
    ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
    ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
    ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
    ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
    ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
    ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
    ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
    ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
    ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
    ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
    ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
    ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f ?
};

unsigned char const mikro_bmp[504] = {
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0, 255, 255,   0, 255, 255, 255,   7,  71,  99,  99,  99,
 99,  99,  99,  99, 103,   7, 143, 255, 255,  63,  63,  31,
 15,  15,   7,   7,   7,   3,   3,   3,   3,   3,   3,   3,
  3,   3,   7,   7,   7,  15,  15,  31,  31,  63, 255, 255,
255, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0, 255, 255,   0, 255, 255, 255,  14,  12,  12,  28,  60,
 60,  60,  60,  60, 252, 254, 255,   3,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0, 248, 254, 254, 254, 252,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
 15, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0, 255, 255,   0, 255, 255, 255,  34, 102,   6,   6,  30,
 14,   6,   6,  34, 255, 255, 255,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   1,   3,   3,   3,   3,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0, 255, 255,   0, 255, 255, 255, 196, 254, 222, 198, 198,
198, 198, 198, 198, 255, 255, 255,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0, 254, 255, 255, 255, 255,
255,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,
  7, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0, 255, 255,   0, 255, 255, 255,  16,  24,  24,  24,  24,
 24,  24,  24,  24,  27,  63, 255, 224, 128,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,  15,  63, 127,  63,  63,
 15,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 128,
240, 255, 255, 255, 255,   0, 255, 255,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  0, 255, 255,   0, 255, 255, 255, 224, 227, 227, 227, 227,
227, 227, 227, 227, 227, 227, 227, 227, 231, 238, 252, 248,
248, 240, 240, 224, 224, 224, 224, 224, 224, 224, 224, 224,
224, 224, 224, 224, 224, 240, 240, 248, 252, 254, 255, 255,
255, 255, 255, 255, 127, 128, 255, 255,   0,   0,   0,   0,
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
#define BL_pin                                                                           0
#define RST_pin                                                                          5
#define CE_pin                                                                           6
#define DC_pin                                                                           7
#define SDO_pin                                                                          8
#define SCK_pin                                                                          9

#define PCD8544_set_Y_addr                                                               0x40
#define PCD8544_set_X_addr                                                               0x80

#define PCD8544_set_temp                                                                 0x04
#define PCD8544_set_bias                                                                 0x10
#define PCD8544_set_VOP                                                                  0x80

#define PCD8544_power_down                                                               0x04
#define PCD8544_entry_mode                                                               0x02
#define PCD8544_extended_instruction                                                     0x01

#define PCD8544_display_blank                                                            0x00
#define PCD8544_display_normal                                                           0x04
#define PCD8544_display_all_on                                                           0x01
#define PCD8544_display_inverted                                                         0x05

#define PCD8544_function_set                                                             0x20
#define PCD8544_display_control                                                          0x08

#define CMD                                                                              0
#define DAT                                                                              1

#define X_max                                                                            84
#define Y_max                                                                            48
#define Rows                                                                             6

#define BLACK                                                                            0
#define WHITE                                                                            1
#define PIXEL_XOR                                                                        2

#define ON                                                                               1
#define OFF                                                                              0

#define NO                                                                               0
#define YES                                                                              1

#define buffer_size                                                                      504


unsigned char PCD8544_buffer[X_max][Rows];


void setup_LCD_GPIOs();
void PCD8544_write(unsigned char type, unsigned char value);
void PCD8544_reset();
void PCD8544_init();
void PCD8544_backlight_state(unsigned char value);
void PCD8544_set_contrast(unsigned char value);
void PCD8544_set_cursor(unsigned char x_pos, unsigned char y_pos);
void PCD8544_print_char(unsigned char ch, unsigned char colour);
void PCD8544_print_custom_char(unsigned char *map);
void PCD8544_fill(unsigned char bufr);
void PCD8544_clear_buffer(unsigned char colour);
void PCD8544_clear_screen(unsigned char colour);
void PCD8544_print_image(const unsigned char *bmp);
void PCD8544_print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour);
void print_char(unsigned char x_pos, unsigned char y_pos, unsigned char ch, unsigned char colour);
void print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour);
void print_chr(unsigned char x_pos, unsigned char y_pos, signed int value, unsigned char colour);
void print_int(unsigned char x_pos, unsigned char y_pos, signed long value, unsigned char colour);
void print_decimal(unsigned char x_pos, unsigned char y_pos, unsigned int value, unsigned char points, unsigned char colour);
void print_float(unsigned char x_pos, unsigned char y_pos, float value, unsigned char points, unsigned char colour);
void Draw_Pixel(unsigned char x_pos, unsigned char y_pos, unsigned char colour);
void Draw_Line(signed int x1, signed int y1, signed int x2, signed int y2, unsigned char colour);
void Draw_Rectangle(signed int x1, signed int y1, signed int x2, signed int y2, unsigned char fill, unsigned char colour);
void Draw_Circle(signed int xc, signed int yc, signed int radius, unsigned char fill, unsigned char colour);
void setup();
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
  /* USER CODE BEGIN 2 */
  unsigned char g = 0;
     const unsigned char txt1[11] = {"MicroArena"};
     const unsigned char txt2[11] = {"SSHAHRYIAR"};

     signed char c = -9;
     signed int i = -66;
     float f = -0.04;

     setup();

     PCD8544_backlight_state(ON);
     //PCD8544_print_image(mikro_bmp);
     HAL_Delay(2000);

     PCD8544_backlight_state(OFF);
     HAL_Delay(2000);

     PCD8544_clear_screen(WHITE);

     PCD8544_backlight_state(ON);
//     Draw_Rectangle(2, 2, 81, 45, OFF, BLACK);
//     HAL_Delay(200);
//
//     Draw_Circle(6, 6, 2, ON, BLACK);
//     HAL_Delay(200);
//     Draw_Circle(77, 6, 2, ON, BLACK);
//     HAL_Delay(200);
//     Draw_Circle(77, 41, 2, ON, BLACK);
//     HAL_Delay(200);
//     Draw_Circle(6, 41, 2, ON, BLACK);
//     HAL_Delay(200);
//
//     Draw_Line(2, 11, 10, 11, BLACK);
//     Draw_Line(73, 11, 81, 11, BLACK);
//     HAL_Delay(200);
//     Draw_Line(2, 36, 10, 36, BLACK);
//     Draw_Line(73, 36, 81, 36, BLACK);
//     HAL_Delay(200);
//     Draw_Line(11, 45, 11, 2, BLACK);
//     Draw_Line(72, 45, 72, 2, BLACK);
     HAL_Delay(200);

     PCD8544_backlight_state(OFF);
     HAL_Delay(400);

     PCD8544_backlight_state(ON);

     for(g = 0; g <= 9; g++)
     {
         PCD8544_set_cursor((18 + (g * 5)), 2);
         PCD8544_print_char(txt1[g], WHITE);
         HAL_Delay(90);
     }

     for(g = 0; g <= 9; g++)
     {
         PCD8544_set_cursor((18 + (g * 5)), 3);
         PCD8544_print_char(txt2[g], WHITE);
         HAL_Delay(90);
     }
        HAL_Delay(4000);

     PCD8544_clear_screen(WHITE);

     PCD8544_print_string(1, 2, "CHR:", WHITE);
     PCD8544_print_string(1, 3, "INT:", WHITE);
     PCD8544_print_string(1, 4, "FLT:", WHITE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
	  print_chr(26, 2, c, WHITE);
	          print_int(26, 3, i, WHITE);
	          print_float(26, 4, f, 2, WHITE);
	          c++;
	          i++;
	          f += 0.01;
	          HAL_Delay(400);
    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin|DC_Pin|LIGHT_Pin|CS_Pin 
                          |SCLK_Pin|MOSI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_Pin DC_Pin LIGHT_Pin CS_Pin 
                           SCLK_Pin MOSI_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DC_Pin|LIGHT_Pin|CS_Pin 
                          |SCLK_Pin|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void setup_LCD_GPIOs()
{
     //enable_GPIOB(true);

//     setup_GPIOB(BL_pin, (output_mode_high_speed | GPIO_PP_output));
//     setup_GPIOB(CE_pin, (output_mode_high_speed | GPIO_PP_output));
//     setup_GPIOB(RST_pin, (output_mode_high_speed | GPIO_PP_output));
//     setup_GPIOB(DC_pin, (output_mode_high_speed | GPIO_PP_output));
//     setup_GPIOB(SDO_pin, (output_mode_high_speed | GPIO_PP_output));
//     setup_GPIOB(SCK_pin, (output_mode_high_speed | GPIO_PP_output));

//     GPIOB_pin_high(BL_pin);
//     GPIOB_pin_high(CE_pin);
//     GPIOB_pin_high(DC_pin);
//     GPIOB_pin_low(RST_pin);
//     GPIOB_pin_low(SDO_pin);
//     GPIOB_pin_low(SCK_pin);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
			HAL_Delay(10);
}


void PCD8544_write(unsigned char type, unsigned char value)
{
     unsigned char s = 0x08;

     if(type != 0)
     {
        // GPIOB_pin_high(DC_pin);
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
     }
     else
     {
        //GPIOB_pin_low(DC_pin);
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
     }

          //GPIOB_pin_low(CE_pin);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);

     while(s > 0)
     {
         //GPIOB_pin_low(SCK_pin);
			     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);

         if((value & 0x80) == 0)
         {
             //GPIOB_pin_low(SDO_pin);
					 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
         }
         else
         {
             //GPIOB_pin_high(SDO_pin);
					  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
         }

         value <<= 1;
         //GPIOB_pin_high(SCK_pin);
				 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
         s--;
     };

     //GPIOB_pin_high(CE_pin);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,1);
}


void PCD8544_reset()
{
//     GPIOB_pin_low(RST_pin);
//     delay_us(100);
//     GPIOB_pin_high(RST_pin);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,0);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,1);
}


void PCD8544_init()
{
    setup_LCD_GPIOs();
    PCD8544_reset();
    PCD8544_write(CMD, (PCD8544_extended_instruction | PCD8544_function_set));
    PCD8544_write(CMD, (PCD8544_set_bias | 0x02));
    PCD8544_set_contrast(0x39);
    PCD8544_write(CMD, PCD8544_set_temp);
    PCD8544_write(CMD, (PCD8544_display_normal | PCD8544_display_control));
    PCD8544_write(CMD, PCD8544_function_set);
    PCD8544_write(CMD, PCD8544_display_all_on);
    PCD8544_write(CMD, PCD8544_display_normal);
    PCD8544_clear_buffer(OFF);
}


void PCD8544_backlight_state(unsigned char value)
{
     if(value != 0)
     {
         //GPIOB_pin_low(BL_pin);
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
     }
     else
     {
        // GPIOB_pin_high(BL_pin);
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
     }
}


void PCD8544_set_contrast(unsigned char value)
{
    if(value >= 0x7F)
    {
       value = 0x7F;
    }

    PCD8544_write(CMD, (PCD8544_extended_instruction | PCD8544_function_set));
    PCD8544_write(CMD, (PCD8544_set_VOP | value));
    PCD8544_write(CMD, PCD8544_function_set);
}


void PCD8544_set_cursor(unsigned char x_pos, unsigned char y_pos)
{
    PCD8544_write(CMD, (PCD8544_set_X_addr | x_pos));
    PCD8544_write(CMD, (PCD8544_set_Y_addr | y_pos));
}


void PCD8544_print_char(unsigned char ch, unsigned char colour)
{
     unsigned char s = 0;
     unsigned char chr = 0;

     for(s = 0; s <= 4; s++)
     {
           chr = font[(ch - 0x20)][s];
           if(colour == BLACK)
           {
               chr = ~chr;
           }
           PCD8544_write(DAT, chr);
     }
}


void PCD8544_print_custom_char(unsigned char *map)
{
    unsigned char s = 0;

    for(s = 0; s <= 4; s++)
    {
        PCD8544_write(DAT, *map++);
    }
}


void PCD8544_fill(unsigned char bufr)
{
    unsigned int s = 0;

    PCD8544_set_cursor(0, 0);

    for(s = 0; s < buffer_size; s++)
    {
        PCD8544_write(DAT, bufr);
    }
}


void PCD8544_clear_buffer(unsigned char colour)
{
    unsigned char x_pos = 0;
    unsigned char y_pos = 0;

    for(x_pos; x_pos < X_max; x_pos++)
    {
        for(y_pos; y_pos < Rows; y_pos++)
        {
            PCD8544_buffer[x_pos][y_pos] = colour;
        }
    }
}


void PCD8544_clear_screen(unsigned char colour)
{
    unsigned char x_pos = 0;
    unsigned char y_pos = 0;

    for(y_pos = 0; y_pos < Rows; y_pos++)
    {
        for(x_pos = 0; x_pos < X_max; x_pos++)
        {
            PCD8544_print_string(x_pos, y_pos, " ", colour);
        }
    }
}

void PCD8544_print_string(unsigned char x_pos, unsigned char y_pos, unsigned char *ch, unsigned char colour)
{
    PCD8544_set_cursor(x_pos, y_pos);

    do
    {
       PCD8544_print_char(*ch++, colour);
    }while((*ch >= 0x20) && (*ch <= 0x7F));
}


void print_chr(unsigned char x_pos, unsigned char y_pos, signed int value, unsigned char colour)
{
    unsigned char ch = 0x00;

    if(value < 0)
    {
        PCD8544_set_cursor(x_pos, y_pos);
        PCD8544_print_char(0x2D, colour);
        value = -value;
    }
    else
    {
        PCD8544_set_cursor(x_pos, y_pos);
        PCD8544_print_char(0x20, colour);
    }

     if((value > 99) && (value <= 999))
     {
         ch = (value / 100);
         PCD8544_set_cursor((x_pos + 6), y_pos);
         PCD8544_print_char((48 + ch), colour);

         ch = ((value % 100) / 10);
         PCD8544_set_cursor((x_pos + 12), y_pos);
         PCD8544_print_char((48 + ch), colour);

         ch = (value % 10);
         PCD8544_set_cursor((x_pos + 18), y_pos);
         PCD8544_print_char((48 + ch), colour);
     }
     else if((value > 9) && (value <= 99))
     {
         ch = ((value % 100) / 10);
         PCD8544_set_cursor((x_pos + 6), y_pos);
         PCD8544_print_char((48 + ch), colour);

         ch = (value % 10);
         PCD8544_set_cursor((x_pos + 12), y_pos);
         PCD8544_print_char((48 + ch), colour);

         PCD8544_set_cursor((x_pos + 18), y_pos);
         PCD8544_print_char(0x20, colour);
     }
     else if((value >= 0) && (value <= 9))
     {
         ch = (value % 10);
         PCD8544_set_cursor((x_pos + 6), y_pos);
         PCD8544_print_char((48 + ch), colour);

         PCD8544_set_cursor((x_pos + 12), y_pos);
         PCD8544_print_char(0x20, colour);

         PCD8544_set_cursor((x_pos + 18), y_pos);
         PCD8544_print_char(0x20, colour);
     }
}


void print_int(unsigned char x_pos, unsigned char y_pos, signed long value, unsigned char colour)
{
    unsigned char ch = 0x00;

    if(value < 0)
    {
        PCD8544_set_cursor(x_pos, y_pos);
        PCD8544_print_char(0x2D, colour);
        value = -value;
    }
    else
    {
        PCD8544_set_cursor(x_pos, y_pos);
        PCD8544_print_char(0x20, colour);
    }

    if(value > 9999)
    {
        ch = (value / 10000);
        PCD8544_set_cursor((x_pos + 6), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = ((value % 10000)/ 1000);
        PCD8544_set_cursor((x_pos + 12), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = ((value % 1000) / 100);
        PCD8544_set_cursor((x_pos + 18), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = ((value % 100) / 10);
        PCD8544_set_cursor((x_pos + 24), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = (value % 10);
        PCD8544_set_cursor((x_pos + 30), y_pos);
        PCD8544_print_char((48 + ch), colour);
    }

    else if((value > 999) && (value <= 9999))
    {
        ch = ((value % 10000)/ 1000);
        PCD8544_set_cursor((x_pos + 6), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = ((value % 1000) / 100);
        PCD8544_set_cursor((x_pos + 12), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = ((value % 100) / 10);
        PCD8544_set_cursor((x_pos + 18), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = (value % 10);
        PCD8544_set_cursor((x_pos + 24), y_pos);
        PCD8544_print_char((48 + ch), colour);

        PCD8544_set_cursor((x_pos + 30), y_pos);
        PCD8544_print_char(0x20, colour);
    }
    else if((value > 99) && (value <= 999))
    {
        ch = ((value % 1000) / 100);
        PCD8544_set_cursor((x_pos + 6), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = ((value % 100) / 10);
        PCD8544_set_cursor((x_pos + 12), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = (value % 10);
        PCD8544_set_cursor((x_pos + 18), y_pos);
        PCD8544_print_char((48 + ch), colour);

        PCD8544_set_cursor((x_pos + 24), y_pos);
        PCD8544_print_char(0x20, colour);

        PCD8544_set_cursor((x_pos + 30), y_pos);
        PCD8544_print_char(0x20, colour);
    }
    else if((value > 9) && (value <= 99))
    {
        ch = ((value % 100) / 10);
        PCD8544_set_cursor((x_pos + 6), y_pos);
        PCD8544_print_char((48 + ch), colour);

        ch = (value % 10);
        PCD8544_set_cursor((x_pos + 12), y_pos);
        PCD8544_print_char((48 + ch), colour);

        PCD8544_set_cursor((x_pos + 18), y_pos);
        PCD8544_print_char(0x20, colour);

        PCD8544_set_cursor((x_pos + 24), y_pos);
        PCD8544_print_char(0x20, colour);

        PCD8544_set_cursor((x_pos + 30), y_pos);
        PCD8544_print_char(0x20, colour);
    }
    else
    {
        ch = (value % 10);
        PCD8544_set_cursor((x_pos + 6), y_pos);
        PCD8544_print_char((48 + ch), colour);

        PCD8544_set_cursor((x_pos + 12), y_pos);
        PCD8544_print_char(0x20, colour);

        PCD8544_set_cursor((x_pos + 18), y_pos);
        PCD8544_print_char(0x20, colour);

        PCD8544_set_cursor((x_pos + 24), y_pos);
        PCD8544_print_char(0x20, colour);

        PCD8544_set_cursor((x_pos + 30), y_pos);
        PCD8544_print_char(0x20, colour);
    }
}

void print_decimal(unsigned char x_pos, unsigned char y_pos, unsigned int value, unsigned char points, unsigned char colour)
{
    unsigned char ch = 0x00;

    PCD8544_set_cursor(x_pos, y_pos);
    PCD8544_print_char(0x2E, colour);

    ch = (value / 1000);
    PCD8544_set_cursor((x_pos + 6), y_pos);
    PCD8544_print_char((48 + ch), colour);

    if(points > 1)
    {
        ch = ((value % 1000) / 100);
        PCD8544_set_cursor((x_pos + 12), y_pos);
        PCD8544_print_char((48 + ch), colour);


        if(points > 2)
        {
            ch = ((value % 100) / 10);
            PCD8544_set_cursor((x_pos + 18), y_pos);
            PCD8544_print_char((48 + ch), colour);

            if(points > 3)
            {
                ch = (value % 10);
                PCD8544_set_cursor((x_pos + 24), y_pos);
                PCD8544_print_char((48 + ch), colour);;
            }
        }
    }
}


void print_float(unsigned char x_pos, unsigned char y_pos, float value, unsigned char points, unsigned char colour)
{
    signed long tmp = 0x00;

    tmp = ((signed long)value);
    print_int(x_pos, y_pos, tmp, colour);
    tmp = ((value - tmp) * 10000);

    if(tmp < 0)
    {
       tmp = -tmp;
    }

    if((value >= 9999) && (value < 99999))
    {
        print_decimal((x_pos + 36), y_pos, tmp, points, colour);
    }
    else if((value >= 999) && (value < 9999))
    {
        print_decimal((x_pos + 30), y_pos, tmp, points, colour);
    }
    else if((value >= 99) && (value < 999))
    {
        print_decimal((x_pos + 24), y_pos, tmp, points, colour);
    }
    else if((value >= 9) && (value < 99))
    {
        print_decimal((x_pos + 18), y_pos, tmp, points, colour);
    }
    else if(value < 9)
    {
        print_decimal((x_pos + 12), y_pos, tmp, points, colour);
        if((value) < 0)
        {
            PCD8544_set_cursor(x_pos, y_pos);
            PCD8544_print_char(0x2D, colour);
        }
        else
        {
            PCD8544_set_cursor(x_pos, y_pos);
            PCD8544_print_char(0x20, colour);
        }
    }
}


void Draw_Pixel(unsigned char x_pos, unsigned char y_pos, unsigned char colour)
{
    unsigned char row = 0;
    unsigned char value = 0;

    if((x_pos < 0) || (x_pos >= X_max) || (y_pos < 0) || (y_pos >= Y_max))
    {
        return;
    }

    row = (y_pos >> 3);

    value = PCD8544_buffer[x_pos][row];

    if(colour == BLACK)
    {
        value |= (1 << (y_pos % 8));
    }
    else if(colour == WHITE)
    {
        value &= (~(1 << (y_pos % 8)));
    }
    else if(colour == PIXEL_XOR)
    {
        value ^= (1 << (y_pos % 8));
    }

    PCD8544_buffer[x_pos][row] = value;

    PCD8544_set_cursor(x_pos, row);
    PCD8544_write(DAT, value);
}
void setup()
{
    PCD8544_init();
    PCD8544_clear_screen(WHITE);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
