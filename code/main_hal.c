/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_CS GPIO_PIN_2
#define LCD_RST GPIO_PIN_1
#define LCD_DC GPIO_PIN_0

#define SCREEN_VERTICAL_1	0
#define SCREEN_HORIZONTAL_1	1
#define SCREEN_VERTICAL_2   2
#define SCREEN_HORIZONTAL_2	3


#define BLACK       0x0000
#define NAVY        0x000F
#define DARKGREEN   0x03E0
#define DARKCYAN    0x03EF
#define MAROON      0x7800
#define PURPLE      0x780F
#define OLIVE       0x7BE0
#define LIGHTGREY   0xC618
#define DARKGREY    0x7BEF
#define BLUE        0x001F
#define GREEN       0x07E0
#define CYAN        0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define ORANGE      0xFD20
#define GREENYELLOW 0xAFE5
#define PINK        0xF81F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
volatile uint16_t LCD_HEIGHT = 240;
volatile uint16_t LCD_WIDTH =  320;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void SPI1_init();

void lcdInit(void);
void ILI9341_Draw_Pixel(uint16_t X,uint16_t Y,uint16_t Colour);
void ILI9341_Set_Rotation(uint8_t Rotation);
void ILI9341_SPI_Send(unsigned char SPI_data);
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
//  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  lcdInit();
  uint16_t x, y;
  x = y = 0;
  while (1)
  {
	  uint8_t r = (rand()/(float)RAND_MAX)*255;
	  uint8_t g = (rand()/(float)RAND_MAX)*255;
	  uint8_t b = (rand()/(float)RAND_MAX)*255;
	  uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
	  ILI9341_Draw_Pixel((rand()/(float)RAND_MAX)*LCD_WIDTH,
			  (rand()/(float)RAND_MAX)*LCD_HEIGHT, color);
	  x++;
	  y++;
	  //HAL_Delay(100);
    /* USER CODE END WHILE */
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)data, (uint8_t*)buffer, strlen(data), 100);
//    HAL_Delay(10);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    //HAL_Delay(4000);
    /* USER CODE BEGIN 3 */
  }
}








void writeCmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 1);
//	ILI9341_SPI_Send((unsigned char)cmd);
//	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);
}

void writeData(uint8_t data)
{
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);
//	GPIOA->CRH
}

void lcdReset(void)
{
	HAL_GPIO_WritePin(GPIOA, LCD_RST, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, LCD_RST, GPIO_PIN_SET);
}

void lcdEnable(void)
{
	HAL_GPIO_WritePin(GPIOA, LCD_RST, GPIO_PIN_SET);
}

void ILI9341_Set_Rotation(uint8_t Rotation)
{

	uint8_t screen_rotation = Rotation;

	writeCmd(0x36);
	HAL_Delay(1);

	switch(screen_rotation)
{
	case SCREEN_VERTICAL_1:
		writeData(0x40|0x08);
		LCD_WIDTH = 240;
		LCD_HEIGHT = 320;
		break;
	case SCREEN_HORIZONTAL_1:
		writeData(0x20|0x08);
		LCD_WIDTH  = 320;
		LCD_HEIGHT = 240;
		break;
	case SCREEN_VERTICAL_2:
		writeData(0x80|0x08);
		LCD_WIDTH  = 240;
		LCD_HEIGHT = 320;
		break;
	case SCREEN_HORIZONTAL_2:
		writeData(0x40|0x80|0x20|0x08);
		LCD_WIDTH  = 320;
		LCD_HEIGHT = 240;
		break;
	default:
		//EXIT IF SCREEN ROTATION NOT VALID!
		break;
	}
}

void lcdInit(void)
{
	lcdEnable();
	MX_SPI1_Init();
//	SPI1_init();
	lcdReset();

	writeCmd(0x01);
	HAL_Delay(1000);

	writeCmd(0x3A);
	writeData(0x55);


	//EXIT SLEEP
	writeCmd(0x11);
	HAL_Delay(120);

	//TURN ON DISPLAY
	writeCmd(0x29);

	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);

}

void ILI9341_SPI_Send(unsigned char SPI_Data)
{
	HAL_SPI_Transmit(&hspi1, &SPI_Data, 1, 1);
//	while (!(SPI1->SR & 2)) {}
////	GPIOB->BSRR = 0x0800;
//	SPI1->DR = SPI_Data;
//	while (SPI1->SR & 0x80) {}
////	GPIOB->BSRR = 0x08000000;
}

void ILI9341_Draw_Pixel(uint16_t X,uint16_t Y,uint16_t Colour)
{
	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;	//OUT OF BOUNDS!

	//ADDRESS
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	ILI9341_SPI_Send(0x2A);
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//XDATA
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	unsigned char Temp_Buffer[4] = {X>>8,X, (X+50)>>8, (X+50)};
	HAL_SPI_Transmit(&hspi1, Temp_Buffer, 4, 1);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//ADDRESS
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	ILI9341_SPI_Send(0x2B);
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//YDATA
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	unsigned char Temp_Buffer1[4] = {Y>>8,Y, (Y+50)>>8, (Y+50)};
	HAL_SPI_Transmit(&hspi1, Temp_Buffer1, 4, 1);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//ADDRESS
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	ILI9341_SPI_Send(0x2C);
	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//COLOUR
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	unsigned char Temp_Buffer2[2] = {Colour>>8, Colour};
	HAL_SPI_Transmit(&hspi1, Temp_Buffer2, 2, 1);
	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void SPI1_init()
{
	// MISO/MOSI/SCK GPIO-SPI config:
//	RCC->APB2ENR |= 0x04; // enable GPIOA clk
//	RCC->APB2ENR |= 0x1000; // enable SPI1 clk
	RCC->APB2ENR |= 0x100d;
//	RCC->APB2ENR |= 1;
//	GPIOA->CRL = 0;
	GPIOA->CRL |= 0xb4b44444; // 0x99900000; // set GPIOA 5,6,7 to High-Speed Output and Alternate-Function mode
	GPIOA->CRH |= 0x88844444;
	// CS, CD, RST config:
	RCC->APB2ENR |= 0x08; // enable GPIOB clk
//	GPIOB->CRL = 0;
//	GPIOB->CRH = 0;
	GPIOB->CRL |= 0x44484424; // set GPIOB 1 to lowspeed output.
	GPIOB->CRH |= 0x44442244; // Pin 10, 11, lowspeed output
//	GPIOB->CRH |= 0x1900; // set GPIOB 10, 11 to highspeed output.

	// SPI1 config-register setup and enable
//	SPI1->CR1 = 0;
	SPI1->CR1 = 0x344; //0x31C; // MSB, 8-Bit, BR 2, Low Polarity, On first Phase
    SPI1->CR2 = 0; // Interrupts disabled
//    SPI1->CR1 |= 0x40; // SPI1 enable
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
//  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_CS|LCD_DC|LCD_RST, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = LCD_CS|LCD_DC|LCD_RST;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* configure SPI Pins */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
