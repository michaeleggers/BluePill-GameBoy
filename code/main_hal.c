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

static const uint8_t font[] = {
    // 0
    0x00, 0xFF, 0xFF, 0xFF, 0x00,
    0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x00, 0xFF, 0xFF, 0xFF, 0x00,
    // 1
    0x00, 0x00, 0x00, 0xFF, 0x00,
    0x00, 0x00, 0xFF, 0xFF, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0x00,
    0x00, 0x00, 0xFF, 0xFF, 0xFF,
    // 2
    0x00, 0xFF, 0xFF, 0xFF, 0x00,
    0xFF, 0x00, 0x00, 0x00, 0xFF,
    0x00, 0x00, 0xFF, 0xFF, 0x00,
    0x00, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF,
    // 3
    0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0xFF, 0x00,
    0x00, 0x00, 0xFF, 0x00, 0x00,
};

static const uint16_t box[] = {
    0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7, 0x61c6, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x8aa7, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x61c6, 0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7
};

static const uint16_t tanya[] = {
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x39c7, 0x39c7, 0x39c7, 0xf1e4, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xee6f, 0xee6f, 0xf1e4, 0x39c7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xee6f, 0xee6f, 0x39c7, 0x39c7, 0x39c7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xee6f, 0x39c7, 0x39c7, 0x39c7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa3fd, 0x5175, 0x39c7, 0x39c7, 0x39c7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa3fd, 0x5175, 0x5175, 0x5175, 0x39c7, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa3fd, 0x5175, 0x5175, 0x5175, 0xa3fd, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0xee6f, 0xa3fd, 0x5175, 0x5175, 0x5175, 0xa3fd, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5175, 0x5175, 0x5175, 0xee6f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x5175, 0x5175, 0x5175, 0x5175, 0x5175, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x5175, 0x5175, 0x5175, 0x5175, 0x5175, 0x5175, 0x5175, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xee6f, 0x0000, 0xee6f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xee6f, 0x0000, 0xee6f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xa986, 0xa986, 0x0000, 0xa986, 0xa986, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
};

static uint8_t dirty_buffer[9600] = { 0 };

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

// for some reason these macros will confuse the display (especially DC_HIGH).
// not sure what is going on but _dont_ use them just yet!
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
void ili9341_print(uint16_t x, uint16_t y, char const * string, uint8_t length);
void ili9341_draw_bitmap(uint16_t x, uint16_t y, uint16_t scale, uint16_t size, uint16_t const * bitmap);
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
    //    HAL_Delay(1);
    asm("nop");
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
    //  ili9341_fill_rect(0, 0, 10, 10, ILI9341_RED);
    //  ili9341_fill_rect(230, 0, 10, 10, ILI9341_RED);
    //  ili9341_fill_rect(0, 310, 10, 10, ILI9341_RED);
    //  ili9341_fill_rect(230, 310, 10, 10, ILI9341_RED);
    //  HAL_Delay(10);
    //  for (int y=0; y < LCD_HEIGHT; ++y)
    //  {
    //	  for (int x=0; x < LCD_WIDTH; ++x)
    //	  {
    //		  uint8_t r = (rand()/(float)RAND_MAX)*255;
    //		  uint8_t g = (rand()/(float)RAND_MAX)*255;
    //		  uint8_t b = (rand()/(float)RAND_MAX)*255;
    //		  uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    //		  ili9341_draw_pixel(x, y, color);
    //	  }
    //  }
    
    //  for (int row = 0; row < 7; ++row)
    //  {
    //	  for (int col = 0; col < 7; ++col)
    //	  {
    //		  ili9341_draw_bitmap(row*32, col*32, 4, box);
    //	  }
    //  }
    int16_t xVel = 1;
    int16_t yVel = 1;
    int16_t xPos = 1;
    int16_t yPos = 1;
    uint16_t scaler = 3;
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
        //	  ili3941_fillscreen(ILI9341_BLACK);
        //	  HAL_Delay(16);
        
        
        //	  ili9341_fill_rect(xPos, yPos, scaler*8, scaler*8, ILI9341_ORANGE);
        //	  ili9341_print(0, 0, "0123", 4);
        
        // remember old bitmap pos for convinience
        uint16_t oldPosX = xPos;
        uint16_t oldPosY = yPos;
        
        // dirt buffer window last frame
        uint16_t dbX = xPos/8;
        uint16_t dbY = 40*yPos;
        for (int i=0; i<8*scaler; ++i) // bitmap 8px high -> go through 8 rows
        {
            uint8_t bitPos = xPos % 8; // bitpos in this column
            uint8_t *dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX] );
            uint16_t bitsLeft = 8*scaler;
            uint16_t xPosTemp = xPos;
            uint16_t byteColumns = scaler+1;
            for (uint16_t column = 0; column < byteColumns; ++column)
            {
                dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX+column] );
                for (int y = bitPos; (y < 8) && (bitsLeft > 0); ++y)
                {
                    *dirtBufferByte |= (0x80 >> y);
                    bitsLeft--;
                    //ili9341_draw_pixel(xPos+column*8 + y, yPos + i, ILI9341_RED);
                }
                xPosTemp += (8-bitPos); // align it to next byte
                bitPos = 0;//xPosTemp % 8;
            }
        }
        
        // bitmap pos update
        xPos += xVel;
	    yPos += yVel;
        if (yPos + 8*scaler  > LCD_HEIGHT) yVel *= -1;
        if (yPos < 0) { yVel *= -1; yPos = 0; }
        if (xPos + 8*scaler > LCD_WIDTH) xVel *= -1;
        if (xPos < 0) { xVel *= -1; xPos = 0; }
        
        // dirt buffer window new frame
        uint16_t dbXnew = xPos/8;
        uint16_t dbYnew = 40*yPos;
        for (int i=0; i<8*scaler; ++i) // bitmap 8px high -> go through 8 rows
        {
            uint8_t bitPos = xPos % 8; // bitpos in this column
            uint8_t *dirtBufferByte = &( dirty_buffer[dbYnew + i*40 + dbXnew] );
            uint16_t bitsLeft = 8*scaler;
            uint16_t xPosTemp = xPos;
            uint16_t byteColumns = scaler+1;
            for (uint16_t column = 0; column < byteColumns; ++column)
            {
                dirtBufferByte = &( dirty_buffer[dbYnew + i*40 + dbXnew+column] );
                for (int y = bitPos; (y < 8) && (bitsLeft > 0); ++y)
                {
                    *dirtBufferByte &= ~(0x80 >> y);
                    bitsLeft--;
                    //ili9341_draw_pixel(xPos+column*8 + y, yPos + i, ILI9341_GREEN);
                }
                xPosTemp += (8-bitPos); // align it to next byte
                bitPos = 0;//xPosTemp % 8;
            }
        }
        
        // finally, draw the actual bitmap at its new location
        ili9341_draw_bitmap(xPos, yPos, scaler, 8, box);
        
        // cleanup: clear the dirt buffer from remaining old frame and draw background color
        for (int i=0; i<8*scaler; ++i) // bitmap 8px high -> go through 8 rows
        {
            uint8_t bitPos = oldPosX % 8; // bitpos in this column
            uint8_t *dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX] );
            uint16_t bitsLeft = 8*scaler;
            uint16_t xPosTemp = oldPosX;
            uint8_t xOffset = 0;
            uint16_t byteColumns = scaler+1;
            for (uint16_t column = 0; column < byteColumns; ++column)
            {
                dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX+column] );
                for (int y = bitPos; (y < 8) && (bitsLeft > 0); ++y)
                {
                    uint8_t dbMask = (0x80 >> y);
                    if ( (*dirtBufferByte & dbMask) == dbMask )
                    {
                        *dirtBufferByte &= ~dbMask;
                        ili9341_draw_pixel(oldPosX + xOffset, oldPosY + i, ILI9341_BLACK);
                    }
                    bitsLeft--;
                    xOffset++;
                }
                xPosTemp += (8-bitPos); // align it to next byte
                bitPos = 0;//xPosTemp % 8;
            }
        }
        
        /*
            for (int i=0; i < 8*scaler; ++i)
            {
                uint8_t *dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX] );
                uint8_t bitPos = oldPosX % 8;
                uint8_t xOffset = 0;
                for (int y = bitPos; y < 8; ++y)
                {
                    uint8_t dbMask = (0x01 << (7 - y));
                    if ( (*dirtBufferByte & dbMask) == dbMask )
                    {
                        *dirtBufferByte &= ~dbMask;
                        ili9341_draw_pixel(oldPosX + xOffset, oldPosY + i, ILI9341_BLACK);
                    }
                    xOffset++;
                }
                if (bitPos)
                {
                    dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX+1] );
                    for (int y = 0; y < bitPos; ++y)
                    {
                        uint8_t dbMask = (0x01 << (7 - y));
                        if ( (*dirtBufferByte & dbMask) == dbMask )
                        {
                            *dirtBufferByte &= ~dbMask;
                            ili9341_draw_pixel(oldPosX + xOffset, oldPosY + i, ILI9341_BLACK);
                        }
                        xOffset++;
                    }
                }
            }
            */
        
        
        //HAL_Delay(30);
        
        //	  for (int row = 0; row < LCD_HEIGHT; row += 20)
        //	  {
        //		  ili9341_fill_rect(0, row, LCD_WIDTH, 1, ILI9341_BLUE);
        //	  }
    }
}

void ili9341_draw_bitmap(uint16_t x, uint16_t y, uint16_t scale, uint16_t size, uint16_t const * bitmap)
{
    if((x + size*scale > LCD_WIDTH) || (y + size*scale > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    
    ili9341_set_window(x, y, size*scale-1, size*scale-1);
    GPIOA->ODR &= ~GPIO_ODR_ODR2;
    for (int row = 0; row < size; ++row)
    {
        for (int s = 0; s < scale; ++s)
            for (int col = 0; col < size*scale; ++col)
        {
            uint16_t pixel = bitmap[row*size + (col / scale)];
            //			if ( pixel != ILI9341_BLACK )
            //			{
            SPI1_write(pixel >> 8);
            SPI1_write(pixel);
            //			}
            //			else
            //			{
            //				SPI1_write(ILI9341_WHITE >> 8);
            //				SPI1_write(ILI9341_WHITE);
            //			}
        }
    }
    GPIOA->ODR |= GPIO_ODR_ODR2;
}

#define FONT_SIZE  5
#define FONT_SCALE 5
void ili9341_print(uint16_t x, uint16_t y, char const * string, uint8_t length)
{
    if((x+FONT_SIZE*FONT_SCALE > LCD_WIDTH) || (y+FONT_SIZE*FONT_SCALE > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    
    uint16_t posOffset = 0;
    while (posOffset < length)
    {
        uint16_t offset = string[posOffset] - 0x30;
        offset *= 25;
        ili9341_set_window(x + posOffset*FONT_SIZE*FONT_SCALE, y, FONT_SIZE*FONT_SCALE-1, FONT_SIZE*FONT_SCALE-1);
        GPIOA->ODR &= ~GPIO_ODR_ODR2;
        for (int row = 0; row < FONT_SIZE; ++ row)
        {
            for (int i = 0; i < FONT_SCALE; ++i)
                for (int col = 0; col < FONT_SIZE*FONT_SCALE; ++col)
            {
                uint16_t fontPixel = font[ offset + (row*FONT_SIZE + (col / FONT_SCALE))  ];
                fontPixel |= (fontPixel << 8);
                SPI1_write(fontPixel >> 8);
                SPI1_write(fontPixel);
            }
        }
        GPIOA->ODR |= GPIO_ODR_ODR2;
        posOffset++;
    }
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
    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
    
}

void ili9341_set_window(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    //ADDRESS COLUMN
    SPI1_writeCmd(0x2A);
    
    
    //XDATA
    SPI1_writeData(x>>8);
    SPI1_writeData(x);
    SPI1_writeData((x+width)>>8);
    SPI1_writeData(x+width);
    
    //ADDRESS PAGE
    SPI1_writeCmd(0x2B);
    
    //YDATA
    SPI1_writeData(y>>8);
    SPI1_writeData(y);
    SPI1_writeData((y+height)>>8);
    SPI1_writeData(y+height);
    
    //ADDRESS memset
    SPI1_writeCmd(0x2C);
}

void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    ili9341_fill_rect(x, y, 1, 1, color);
}

void ili3941_fillscreen(uint16_t color)
{
    ili9341_set_window(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);
    
    //ADDRESS
    SPI1_writeCmd(0x2C);
    
    GPIOA->ODR &= ~GPIO_ODR_ODR2;
    for (int i=0; i < LCD_WIDTH*LCD_HEIGHT; i++)
    {
        SPI1_write(color >> 8);
        SPI1_write(color);
        //		HAL_Delay(1);
    }
    GPIOA->ODR |= GPIO_ODR_ODR2;
    
}

void ili9341_fill_rect(uint16_t x,uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    if((x+width > LCD_WIDTH) || (y+height > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    ili9341_set_window(x, y, width-1, height-1);
    
    GPIOA->ODR &= ~GPIO_ODR_ODR2;
    for (int i = 0; i < width*height; ++i)
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
