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

#include "ili9341.h"

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

#define CS_HIGH   (GPIOA->ODR |= GPIO_ODR_ODR2)
#define CS_LOW    (GPIOA->ODR &= ~GPIO_ODR_ODR2)
#define DC_HIGH   (GPIOB->ODR |=  GPIO_ODR_ODR0)
#define DC_LOW    (GPIOA->ODR &= ~GPIO_ODR_ODR0)
#define RST_HIGH  (GPIOA->ODR |= GPIO_ODR_ODR1)
#define RST_LOW   (GPIOA->ODR &= ~GPIO_ODR_ODR1)

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

void lcdInit(void);
void ili9341_set_window(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
void ili9341_fill_rect(uint16_t x,uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void ili3941_fillscreen(uint16_t color);
void ILI9341_Set_Rotation(uint8_t Rotation);
uint8_t ILI9341_SPI_Send(unsigned char data);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

static inline uint8_t SPI1_write(uint8_t data)
{
	while (!(SPI1->SR & SPI_SR_TXE)) {}
    *((volatile uint8_t*)&SPI1->DR) = data; // cast is necessary cause DR is uint16_t
    while (SPI1->SR & SPI_SR_BSY) {}
    return SPI1->DR;
}

static inline void SPI1_writeData(uint8_t data)
{
//	GPIOA->ODR |= GPIO_ODR_ODR0;
    GPIOA->ODR &= ~GPIO_ODR_ODR2;
	SPI1_write(data);
	GPIOA->ODR |= GPIO_ODR_ODR2;
}

static inline void SPI1_writeCmd(uint8_t cmd)
{
    GPIOA->ODR &= ~GPIO_ODR_ODR2;
    GPIOA->ODR &= ~GPIO_ODR_ODR0;
    HAL_Delay(1);
    SPI1_write(cmd);
	GPIOA->ODR |= GPIO_ODR_ODR0;
	GPIOA->ODR |= GPIO_ODR_ODR2;

}

int main(void)
{

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  lcdInit();

	  ili3941_fillscreen(ILI9341_BLACK);
	  for (int y=0; y < LCD_HEIGHT; ++y)
	  {
		  for (int x=0; x < LCD_WIDTH; ++x)
		  {
			  uint8_t r = (rand()/(float)RAND_MAX)*255;
			  uint8_t g = (rand()/(float)RAND_MAX)*255;
			  uint8_t b = (rand()/(float)RAND_MAX)*255;
			  uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
			  ili9341_draw_pixel(x, y, color);
		  }
	  }
  while (1)
  {
//	  ili3941_fillscreen(ILI9341_RED);
//	  ili3941_fillscreen(ILI9341_GREEN);
//	  ili3941_fillscreen(ILI9341_BLUE);
//	  ili3941_fillscreen(ILI9341_MAGENTA);
//	  ili3941_fillscreen(ILI9341_ORANGE);
//	  ili3941_fillscreen(ILI9341_YELLOW);
//	  ili3941_fillscreen(ILI9341_BLACK);
//	  ili3941_fillscreen(ILI9341_WHITE);
//	  for (int i=0; i<1000; i++)
//	  {
//		  uint8_t r = (rand()/(float)RAND_MAX)*255;
//		  uint8_t g = (rand()/(float)RAND_MAX)*255;
//		  uint8_t b = (rand()/(float)RAND_MAX)*255;
//		  uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
//		  ili9341_fill_rect((rand()/(float)RAND_MAX)*LCD_WIDTH, (rand()/(float)RAND_MAX)*LCD_HEIGHT,
//				  (rand()/(float)RAND_MAX)*LCD_WIDTH, (rand()/(float)RAND_MAX)*LCD_HEIGHT, color);
//	  }
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

	SPI1_writeCmd(0x36);
	HAL_Delay(1);

	switch(screen_rotation)
{
	case SCREEN_VERTICAL_1:
		SPI1_writeData(0x40|0x08);
		LCD_WIDTH = 240;
		LCD_HEIGHT = 320;
		break;
	case SCREEN_HORIZONTAL_1:
		SPI1_writeData(0x20|0x08);
		LCD_WIDTH  = 320;
		LCD_HEIGHT = 240;
		break;
	case SCREEN_VERTICAL_2:
		SPI1_writeData(0x80|0x08);
		LCD_WIDTH  = 240;
		LCD_HEIGHT = 320;
		break;
	case SCREEN_HORIZONTAL_2:
		SPI1_writeData(0x40|0x80|0x20|0x08);
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
	SPI1->CR1 |= SPI_CR1_SPE;
/////////////////////////////////////
//	 WORSK PARTIALLY
//	SPI1_writeCmd(0x01);
//	HAL_Delay(1000);
//
//	SPI1_writeCmd(0x3A);
////	HAL_Delay(5);
//	SPI1_writeData(0x55);
//
//
//	//EXIT SLEEP
//	SPI1_writeCmd(0x11);
//	HAL_Delay(120);
//
//	//TURN ON DISPLAY
//	SPI1_writeCmd(0x29);
//	HAL_Delay(120);
// ! WORKS PARTIALLY
///////////////////////////////////////

	//SOFTWARE RESET
	SPI1_writeCmd(0x01);
	HAL_Delay(1000);

	//POWER CONTROL A
	SPI1_writeCmd(0xCB);
	SPI1_writeData(0x39);
	SPI1_writeData(0x2C);
	SPI1_writeData(0x00);
	SPI1_writeData(0x34);
	SPI1_writeData(0x02);

	//POWER CONTROL B
	SPI1_writeCmd(0xCF);
	SPI1_writeData(0x00);
	SPI1_writeData(0xC1);
	SPI1_writeData(0x30);

	//DRIVER TIMING CONTROL A
	SPI1_writeCmd(0xE8);
	SPI1_writeData(0x85);
	SPI1_writeData(0x00);
	SPI1_writeData(0x78);

	//DRIVER TIMING CONTROL B
	SPI1_writeCmd(0xEA);
	SPI1_writeData(0x00);
	SPI1_writeData(0x00);

	//POWER ON SEQUENCE CONTROL
	SPI1_writeCmd(0xED);
	SPI1_writeData(0x64);
	SPI1_writeData(0x03);
	SPI1_writeData(0x12);
	SPI1_writeData(0x81);

	//PUMP RATIO CONTROL
	SPI1_writeCmd(0xF7);
	SPI1_writeData(0x20);

	//POWER CONTROL,VRH[5:0]
	SPI1_writeCmd(0xC0);
	SPI1_writeData(0x23);

	//POWER CONTROL,SAP[2:0];BT[3:0]
	SPI1_writeCmd(0xC1);
	SPI1_writeData(0x10);

	//VCM CONTROL
	SPI1_writeCmd(0xC5);
	SPI1_writeData(0x3E);
	SPI1_writeData(0x28);

	//VCM CONTROL 2
	SPI1_writeCmd(0xC7);
	SPI1_writeData(0x86);

	//MEMORY ACCESS CONTROL
	SPI1_writeCmd(0x36);
	SPI1_writeData(0x48);

	//PIXEL FORMAT
	SPI1_writeCmd(0x3A);
	SPI1_writeData(0x55);

	//FRAME RATIO CONTROL, STANDARD RGB COLOR
	SPI1_writeCmd(0xB1);
	SPI1_writeData(0x00);
	SPI1_writeData(0x18);

	//DISPLAY FUNCTION CONTROL
	SPI1_writeCmd(0xB6);
	SPI1_writeData(0x08);
	SPI1_writeData(0x82);
	SPI1_writeData(0x27);

	//3GAMMA FUNCTION DISABLE
	SPI1_writeCmd(0xF2);
	SPI1_writeData(0x00);

	//GAMMA CURVE SELECTED
	SPI1_writeCmd(0x26);
	SPI1_writeData(0x01);

	//POSITIVE GAMMA CORRECTION
	SPI1_writeCmd(0xE0);
	SPI1_writeData(0x0F);
	SPI1_writeData(0x31);
	SPI1_writeData(0x2B);
	SPI1_writeData(0x0C);
	SPI1_writeData(0x0E);
	SPI1_writeData(0x08);
	SPI1_writeData(0x4E);
	SPI1_writeData(0xF1);
	SPI1_writeData(0x37);
	SPI1_writeData(0x07);
	SPI1_writeData(0x10);
	SPI1_writeData(0x03);
	SPI1_writeData(0x0E);
	SPI1_writeData(0x09);
	SPI1_writeData(0x00);

	//NEGATIVE GAMMA CORRECTION
	SPI1_writeCmd(0xE1);
	SPI1_writeData(0x00);
	SPI1_writeData(0x0E);
	SPI1_writeData(0x14);
	SPI1_writeData(0x03);
	SPI1_writeData(0x11);
	SPI1_writeData(0x07);
	SPI1_writeData(0x31);
	SPI1_writeData(0xC1);
	SPI1_writeData(0x48);
	SPI1_writeData(0x08);
	SPI1_writeData(0x0F);
	SPI1_writeData(0x0C);
	SPI1_writeData(0x31);
	SPI1_writeData(0x36);
	SPI1_writeData(0x0F);

	//EXIT SLEEP
	SPI1_writeCmd(0x11);
	HAL_Delay(120);

//	TURN ON DISPLAY
	SPI1_writeCmd(0x29);

	//STARTING ROTATION
	ILI9341_Set_Rotation(SCREEN_VERTICAL_1);

}

uint8_t ILI9341_SPI_Send(unsigned char data)
{
	HAL_SPI_Transmit(&hspi1, &data, 1, 1);
//	while (!(SPI1->SR & SPI_SR_TXE)) {}
//	*((volatile uint8_t*)&SPI1->DR) = data; // cast is necessary cause DR is uint16_t
//	while (SPI1->SR & SPI_SR_BSY) {}
//	return SPI1->DR;
}

void ili9341_set_window(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
	//ADDRESS
//	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
	SPI1_writeCmd(0x2A);
//	SPI1_write(0x2A);
//	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//XDATA
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
//	unsigned char Temp_Buffer[4] = {X>>8,X, (X+5)>>8, (X+5)};
	SPI1_writeData(x>>8);
	SPI1_writeData(x);
	SPI1_writeData((x+width)>>8);
	SPI1_writeData(x+width);
//	HAL_SPI_Transmit(&hspi1, Temp_Buffer, 4, 1);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//ADDRESS
//	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
//	DC_LOW;
//	CS_LOW;
//	SPI1_write(0x2B);
	SPI1_writeCmd(0x2B);
//	SPI1_writeCmd(0x2B);
//	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);
//	DC_HIGH;
//	CS_HIGH;

	//YDATA
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
//	unsigned char Temp_Buffer1[4] = {Y>>8,Y, (Y+5)>>8, (Y+5)};
//	HAL_SPI_Transmit(&hspi1, Temp_Buffer1, 4, 1);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);
	SPI1_writeData(y>>8);
	SPI1_writeData(y);
	SPI1_writeData((y+height)>>8);
	SPI1_writeData(y+height);

	//ADDRESS
//	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
//	ILI9341_SPI_Send(0x2C);
	SPI1_writeCmd(0x2C);
//	HAL_GPIO_WritePin(GPIOA, LCD_DC, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

	//COLOUR
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_RESET);
//	unsigned char Temp_Buffer2[2] = {Colour>>8, Colour};
//	HAL_SPI_Transmit(&hspi1, Temp_Buffer2, 2, 1);
}

void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
	if((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) return;	//OUT OF BOUNDS!
	ili9341_set_window(x, y, x+1, y+1);
	GPIOA->ODR &= ~GPIO_ODR_ODR2;
	SPI1_write(color >> 8);
	SPI1_write(color);
	GPIOA->ODR |= GPIO_ODR_ODR2;
}

void ili3941_fillscreen(uint16_t color)
{
	ili9341_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);

	//ADDRESS
	SPI1_writeCmd(0x2C);

	GPIOA->ODR &= ~GPIO_ODR_ODR2;
	for (int i=0; i < 320*240; i++)
	{
		SPI1_write(color >> 8);
		SPI1_write(color);
//		HAL_Delay(1);
	}
	GPIOA->ODR |= GPIO_ODR_ODR2;

}

void ili9341_fill_rect(uint16_t x,uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
	if((x+width >=LCD_WIDTH) || (y+height >=LCD_HEIGHT)) return;	//OUT OF BOUNDS!
	ili9341_set_window(x, y, width, height);

	GPIOA->ODR &= ~GPIO_ODR_ODR2;
	for (int i=0; i<width*height;++i)
	{
		SPI1_write(color>>8);
		SPI1_write(color);
//		HAL_Delay(1);
	}
	GPIOA->ODR |= GPIO_ODR_ODR2;
//	HAL_GPIO_WritePin(GPIOA, LCD_CS, GPIO_PIN_SET);

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
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB busses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }

	// enable super fast 72MHz as SYSCLK. SPEEEEEEEEEED!!!!

	RCC->CR &= ~RCC_CR_HSEON;
	RCC->CR |=  RCC_CR_HSEON;
	while ( !(RCC->CR & RCC_CR_HSERDY)) {}

//	RCC->CFGR &= ~RCC_CFGR_SW; // set HSI as SYSCLK while configuring PLL (is this necessary?)
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler: divide by 2, so APB1 clk will be 36MHz (should not exceed that!)
//	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 prescaler: no divide -> 72MHz
	RCC->CFGR |= RCC_CFGR_PLLSRC; // use HSE as PLL source
//	RCC->CFGR &= ~RCC_CFGR_PLLXTPRE; // don't divide HSE clk
	RCC->CFGR |= RCC_CFGR_PLLMULL9; // clk * 9
	RCC->CR   |= RCC_CR_PLLON; // turn PLL on
	while ( !(RCC->CR & RCC_CR_PLLRDY) ) {} // wait till PLL is ready
	RCC->CFGR &= ~0xF0; // AHB prescaler: SYSCLK not devided.

	// flash wait statess
	FLASH->ACR |= FLASH_ACR_LATENCY_1;

	RCC->CFGR &= ~RCC_CFGR_SW; // select PLL as SYSCLK
	RCC->CFGR |= RCC_CFGR_SW_PLL;

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
//  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    Error_Handler();
//  }

		SPI1->CR1 &= ~SPI_CR1_SPE; // enable SPI
//	    	RCC->APB2ENR |= RCC_APB2RSTR_SPI1RST;
//	    	RCC->APB2ENR &= ~(RCC_APB2RSTR_SPI1RST);
		// configure and enable
		SPI1->CR1 &= ~(SPI_CR1_CPOL|SPI_CR1_CPHA); // pol high, capture on second edge
		SPI1->CR1 &= ~(SPI_CR1_BR);
		SPI1->CR1 &= ~(SPI_CR1_DFF|SPI_CR1_LSBFIRST);
		SPI1->CR1 |= (SPI_CR1_MSTR|SPI_CR1_SSI|SPI_CR1_SSM);
	//	SPI1->CR2 |= SPI_CR2_SSOE;
		SPI1->CR1 |= SPI_CR1_SPE;
		// 8Bit, MSB and BR=2 _should_ be set by default
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
////  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, LCD_CS|LCD_DC|LCD_RST, GPIO_PIN_RESET);
//
//  /*Configure GPIO pins : PB1 PB10 PB11 */
//  GPIO_InitStruct.Pin = LCD_CS|LCD_DC|LCD_RST;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /* configure SPI Pins */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5, GPIO_PIN_RESET);
//  GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
//  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull      = GPIO_NOPULL;
//  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
//  // GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /////////////////////////////////////////////////////////////

  // CMSIS INIT
	// clock enable for SPI1, GPIOA and AF
	RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN|RCC_APB2ENR_IOPAEN|RCC_APB2ENR_SPI1EN);

	// GPIO DC
	GPIOA->CRL |= GPIO_CRL_MODE0_1;
	GPIOA->CRL &= ~( GPIO_CRL_MODE0_0|GPIO_CRL_CNF0 );
	// GPIO RST
	GPIOA->CRL |= GPIO_CRL_MODE1_1;
	GPIOA->CRL &= ~( GPIO_CRL_MODE1_0|GPIO_CRL_CNF1 );
	// GPIO CS
	GPIOA->CRL |= GPIO_CRL_MODE2_1;
	GPIOA->CRL &= ~( GPIO_CRL_MODE2_0|GPIO_CRL_CNF2 );

	// SPI1 SCK, MOSI
	GPIOA->CRL |= (GPIO_CRL_MODE5|GPIO_CRL_MODE7); // both to output PP
	GPIOA->CRL |= (GPIO_CRL_CNF5_1|GPIO_CRL_CNF7_1);
	GPIOA->CRL &= ~( GPIO_CRL_CNF5_0|GPIO_CRL_CNF7_0 );
	// SPI1 MISO
	GPIOA->CRL &= ~( GPIO_CRL_MODE6|GPIO_CRL_CNF6_1 );
	GPIOA->CRL |= GPIO_CRL_CNF6_0;

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
