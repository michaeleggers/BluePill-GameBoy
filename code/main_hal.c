
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

static int16_t gImage_box[] = {
    0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7, 0x61c6, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x8aa7, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x61c6, 0x61c6, 0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x61c6, 0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x8aa7, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x61c6, 0x8aa7,
    0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7, 0x8aa7
};

static const int16_t tanya[] = {
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

static uint8_t gImage_stone_floor[512] = {
    0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X84,0X10,
    0X84,0X10,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X7B,0XCF,0X7B,0XCF,0X7B,0XCF,0X84,0X10,
    0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X7B,0XCF,0X7B,0XCF,0X84,0X10,0X84,0X10,0X84,0X10,
    0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X7B,0XCF,0X7B,0XCF,0X84,0X10,0X84,0X10,0X6B,0X4D,0X6B,0X4D,
    0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X6B,0X4D,0X83,0XF0,0X84,0X10,0X84,0X10,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,
    0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,
    0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X7B,0XCF,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X7B,0XCF,0X6B,0X4D,0X7B,0XCF,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X7B,0XCF,0X7B,0XCF,0X6B,0X4D,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X7B,0XCF,0X7B,0XCF,0X6B,0X4D,0X84,0X10,0X6B,0X4D,
    0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X7B,0XCF,0X7B,0XCF,0X84,0X10,0X6B,0X4D,0X6B,0X4D,
    0X83,0XF0,0X83,0XF0,0X7B,0XCF,0X7B,0XCF,0X6B,0X4D,0X84,0X10,0X6B,0X4D,0X6B,0X4D,
    0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X7B,0XCF,0X84,0X10,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,
    0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X84,0X10,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,
    0X6B,0X4D,0X6B,0X4D,0X84,0X10,0X84,0X10,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,
    0X6B,0X4D,0X84,0X10,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X84,0X10,0X84,0X10,0X84,0X10,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X84,0X10,0X84,0X10,0X84,0X10,0X84,0X10,0X6B,0X4D,
    0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,0X83,0XF0,
    0X83,0XF0,0X83,0XF0,0X83,0XF0,0X6B,0X4D,0X6B,0X4D,0X6B,0X4D,0X83,0XF0,0X6B,0X4D
};

static uint8_t map[70] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0B, 0x0B, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0B, 0x0B, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B,
};

static uint8_t dirty_buffer[9600] = { 0 };

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

volatile int16_t LCD_HEIGHT = 240;
volatile int16_t LCD_WIDTH =  320;

typedef enum BitmapByteConfig
{
    BMP_RGB565_16,
    BMP_RGB565_8
} BitmapByteConfig;

typedef struct Bitmap_t
{
    int16_t xPos, yPos;
    int16_t xPosOld, yPosOld;
    int16_t width;
    int16_t height;
    int16_t scale;
    int16_t xVel; // NOTE(Michael): move later into Entity_t or something like that
    int16_t yVel;
    void *color;
    BitmapByteConfig config;
} Bitmap_t;

typedef struct Pipe_t
{
    int16_t xPos, yPos;
    int16_t width, height;
    int16_t xPosOld;
} Pipe_t;

typedef struct Pipepair_t
{
    Pipe_t upper;
    Pipe_t lower;
} Pipepair_t;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void ADC1_init(void);
void lcdInit(void);
void ili9341_set_window(int16_t x, int16_t y, int16_t width, int16_t height);
void ili9341_draw_pixel(int16_t x, int16_t y, int16_t color);
void ili9341_fill_rect(int16_t x, int16_t y, int16_t width, int16_t height, int16_t color);
void ili3941_fillscreen(int16_t color);
void ILI9341_Set_Rotation(uint8_t Rotation);
void ili9341_print(int16_t x, int16_t y, char const * string, uint8_t length);
void ili9341_draw_bitmap(Bitmap_t * bitmap);
void draw_static_bitmap(Bitmap_t * bitmap, int16_t xPos, int16_t yPos, int16_t scale);

static inline uint8_t SPI1_write(uint8_t data)
{
	while (!(SPI1->SR & SPI_SR_TXE)) {}
    *((volatile uint8_t*)&SPI1->DR) = data; // cast is necessary cause DR is int16_t
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
    //ADC1_init();
    lcdInit();
    
    ili3941_fillscreen(ILI9341_BLUE);
    
#define PIPE_PAIR_COUNT 1
    Pipepair_t pipePairs[PIPE_PAIR_COUNT];
    Pipe_t pipe1 = { 
        320, 0,
        20, 100,
        320
    };
    
    Pipe_t pipe2 = { 
        320, 140,
        20, 100,
        320
    };
    Pipe_t pipe3 = pipe1;
    pipe3.xPos += 200;
    Pipe_t pipe4 = pipe2;
    pipe4.xPos += 200;
    Pipepair_t pipePair = {
        pipe1,
        pipe2,
    };
    Pipepair_t pipePair2 = {
        pipe3,
        pipe4
    };
    //pipePairs[0] = pipePair;
    pipePairs[0] = pipePair2;
    
    Bitmap_t stone_floor = {
        0, 0,
        0, 0,
        16, 16,
        2,
        0, 0,
        gImage_stone_floor,
        BMP_RGB565_8
    };
    
    Bitmap_t box = {
        30, 120,
        0, 0,
        8, 8,
        5,
        0, 0,
        gImage_box,
        BMP_RGB565_16
    };
    
    /*
    // draw map
    for (int row = 0; row < 7; ++row)
    {
        for (int col = 0; col < 10; ++col)
        {
            uint8_t tile = map[10*row + col];
            if (tile == 0x00)
            {
                draw_static_bitmap(&stone_floor, 32*col, 32*row, 2);
            }
            else if (tile == 0x0B)
            {
                draw_static_bitmap(&box, 32*col, 32*row, 4);
            }
        }
    }
    */
    //ili9341_fill_rect(0, 0, 100, 100, ILI9341_GREEN);
    
    while (1)
    {
        uint8_t buttonState = (GPIOA->IDR & GPIO_IDR_IDR8) == GPIO_IDR_IDR8;
        if (buttonState)
        {
            //box.yPos -= 5;
        }
        ili9341_draw_bitmap(&box);
        for (int i = 0; i < PIPE_PAIR_COUNT; ++i)
        {
            Pipepair_t *pipePair = &pipePairs[i];
            Pipe_t *upper = &pipePair->upper;
            Pipe_t *lower = &pipePair->lower;
            // upper
            upper->xPosOld = upper->xPos;
            upper->xPos -= 1;
            int16_t xClear = upper->xPos + upper->width;
            int16_t widthClear = upper->xPosOld - upper->xPos;
            ili9341_fill_rect(xClear, upper->yPos, widthClear, upper->height, ILI9341_BLUE);
            ili9341_fill_rect(upper->xPos, upper->yPos, upper->width, upper->height, ILI9341_GREEN);
            // lower
            lower->xPosOld = lower->xPos;
            lower->xPos -= 1;
            int16_t xClearLower = lower->xPos + lower->width;
            int16_t widthClearLower = lower->xPosOld - lower->xPos;
            ili9341_fill_rect(xClearLower, lower->yPos, widthClearLower, lower->height, ILI9341_BLUE);
            ili9341_fill_rect(lower->xPos, lower->yPos, lower->width, lower->height, ILI9341_GREEN);
            if ( (upper->xPos+upper->width) <= 0 )
            {
                ili9341_fill_rect(0, upper->yPos, 1, upper->height, ILI9341_BLUE);
                upper->xPos = 320;
                ili9341_fill_rect(0, lower->yPos, 1, lower->height, ILI9341_BLUE);
                lower->xPos = 320;
                // create new hole
                int16_t holeY = ((float)rand()/(float)RAND_MAX) * 200; // 40 is hole height!!
                upper->height = holeY;
                lower->yPos = holeY+40;
                lower->height = 240 - lower->yPos;
            }
        }
        
        //box.yPos++;
        // read joystick.
        // NOTE(Michael): IDRx can only be read in word mode (32bits)
        //ADC1->CR2 |= ADC_CR2_SWSTART;
        //while ( ADC1->SR & ADC_SR_EOC ) {}
        //int16_t conversion = ADC1->DR;
        
        HAL_Delay(5);
    }
}

/* bitmap width, height MUST be a multiple of 8 pixels!!! */
void ili9341_draw_bitmap(Bitmap_t * bitmap)
{
    int16_t xPos = bitmap->xPos;
    int16_t yPos = bitmap->yPos;
    int16_t xPosOld = bitmap->xPosOld;
    int16_t yPosOld = bitmap->yPosOld;
    int16_t width = bitmap->width;
    int16_t height = bitmap->height;
    int16_t scale = bitmap->scale;
    BitmapByteConfig config = bitmap->config;
    
    // dirt buffer window last frame
    int16_t dbX = xPosOld/8;
    int16_t dbY = 40*yPosOld;
    for (int16_t i=0; i<height*scale; ++i) // bitmap 8px high -> go through 8 rows
    {
        uint8_t bitPos = xPosOld % 8; // bitpos in this column
        uint8_t *dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX] );
        int16_t bitsLeft = width*scale;
        int16_t byteColumns = (scale*width/8)+1;
        for (int16_t column = 0; column < byteColumns; ++column)
        {
            dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX+column] );
            for (int y = bitPos; (y < 8) && (bitsLeft > 0); ++y)
            {
                *dirtBufferByte |= (0x80 >> y);
                bitsLeft--;
                //ili9341_draw_pixel(xPos+column*8 + y, yPos + i, ILI9341_RED);
            }
            bitPos = 0; // TODO(Michael): This only makes sense after first loop! Fix it (some day)!
        }
    }
    
    // dirt buffer window new frame
    int16_t dbXnew = xPos/8;
    int16_t dbYnew = 40*yPos;
    for (int16_t i=0; i<height*scale; ++i) // bitmap 8px high -> go through 8 rows
    {
        uint8_t bitPos = xPos % 8; // bitpos in this column
        uint8_t *dirtBufferByte = &( dirty_buffer[dbYnew + i*40 + dbXnew] );
        int16_t bitsLeft = width*scale;
        int16_t byteColumns = (scale*width/8)+1;
        for (int16_t column = 0; column < byteColumns; ++column)
        {
            dirtBufferByte = &( dirty_buffer[dbYnew + i*40 + dbXnew+column] );
            for (int y = bitPos; (y < 8) && (bitsLeft > 0); ++y)
            {
                *dirtBufferByte &= ~(0x80 >> y);
                bitsLeft--;
                //ili9341_draw_pixel(xPos+column*8 + y, yPos + i, ILI9341_GREEN);
            }
            bitPos = 0;
        }
    }
    
    // cleanup: clear the dirt buffer from remaining old frame and draw background color
    for (int i=0; i<height*scale; ++i) // bitmap 8px high -> go through 8 rows
    {
        uint8_t bitPos = xPosOld % 8; // bitpos in this column
        uint8_t *dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX] );
        int16_t bitsLeft = width*scale;
        uint8_t xOffset = 0;
        int16_t byteColumns = (scale*width/8)+1;
        for (int16_t column = 0; column < byteColumns; ++column)
        {
            dirtBufferByte = &( dirty_buffer[dbY + i*40 + dbX+column] );
            for (int y = bitPos; (y < 8) && (bitsLeft > 0); ++y)
            {
                uint8_t dbMask = (0x80 >> y);
                if ( (*dirtBufferByte & dbMask) == dbMask )
                {
                    *dirtBufferByte &= ~dbMask;
                    ili9341_draw_pixel(xPosOld + xOffset, yPosOld + i, ILI9341_BLUE);
                }
                bitsLeft--;
                xOffset++;
            }
            bitPos = 0;
        }
    }
    
    // draw to device
    if((xPos + width*scale > LCD_WIDTH) || (yPos + height*scale > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    ili9341_set_window(xPos, yPos, width*scale-1, height*scale-1);
    
    GPIOA->ODR &= ~GPIO_ODR_ODR2;
    switch (config)
    {
        case BMP_RGB565_16:
        {
            int16_t *color = (int16_t *)(bitmap->color);
            for (int row = 0; row < height; ++row)
            {
                for (int s = 0; s < scale; ++s)
                    for (int col = 0; col < width*scale; ++col)
                {
                    int16_t pixel = color[row*width + (col / scale)];
                    SPI1_write(pixel >> 8);
                    SPI1_write(pixel);
                }
            }
        }
        break;
        
        case BMP_RGB565_8:
        {
            uint8_t *color = (uint8_t *)(bitmap->color);
            for (int row = 0; row < height; ++row)
            {
                for (int s = 0; s < scale; ++s)
                    for (int col = 0; col < 2*width; col += 2)
                {
                    uint8_t pixelH = color[row*width*2 + col];
                    uint8_t pixelL = color[row*width*2 + col + 1];
                    for (int k = 0; k < scale; ++k)
                    {
                        SPI1_write(pixelH);
                        SPI1_write(pixelL);
                    }
                }
            }
        }
        break;
    }
    GPIOA->ODR |= GPIO_ODR_ODR2;
    
    // remember old bitmap pos for convinience
    bitmap->xPosOld = bitmap->xPos;
    bitmap->yPosOld = bitmap->yPos;
}

void draw_static_bitmap(Bitmap_t * bitmap, int16_t xPos, int16_t yPos, int16_t scale)
{
    int16_t width = bitmap->width;
    int16_t height = bitmap->height;
    BitmapByteConfig config = bitmap->config;
    
    // draw to device
    if((xPos + width*scale > LCD_WIDTH) || (yPos + height*scale > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    ili9341_set_window(xPos, yPos, width*scale-1, height*scale-1);
    
    GPIOA->ODR &= ~GPIO_ODR_ODR2;
    switch (config)
    {
        case BMP_RGB565_16:
        {
            int16_t *color = (int16_t *)(bitmap->color);
            for (int row = 0; row < height; ++row)
            {
                for (int s = 0; s < scale; ++s)
                    for (int col = 0; col < width*scale; ++col)
                {
                    int16_t pixel = color[row*width + (col / scale)];
                    SPI1_write(pixel >> 8);
                    SPI1_write(pixel);
                }
            }
        }
        break;
        
        case BMP_RGB565_8:
        {
            uint8_t *color = (uint8_t *)(bitmap->color);
            for (int row = 0; row < height; ++row)
            {
                for (int s = 0; s < scale; ++s)
                    for (int col = 0; col < 2*width; col += 2)
                {
                    uint8_t pixelH = color[row*width*2 + col];
                    uint8_t pixelL = color[row*width*2 + col + 1];
                    for (int k = 0; k < scale; ++k)
                    {
                        SPI1_write(pixelH);
                        SPI1_write(pixelL);
                    }
                }
            }
        }
        break;
    }
    GPIOA->ODR |= GPIO_ODR_ODR2;
}

#define FONT_SIZE  5
#define FONT_SCALE 5
void ili9341_print(int16_t x, int16_t y, char const * string, uint8_t length)
{
    if((x+FONT_SIZE*FONT_SCALE > LCD_WIDTH) || (y+FONT_SIZE*FONT_SCALE > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    
    int16_t posOffset = 0;
    while (posOffset < length)
    {
        int16_t offset = string[posOffset] - 0x30;
        offset *= 25;
        ili9341_set_window(x + posOffset*FONT_SIZE*FONT_SCALE, y, FONT_SIZE*FONT_SCALE-1, FONT_SIZE*FONT_SCALE-1);
        GPIOA->ODR &= ~GPIO_ODR_ODR2;
        for (int row = 0; row < FONT_SIZE; ++ row)
        {
            for (int i = 0; i < FONT_SCALE; ++i)
                for (int col = 0; col < FONT_SIZE*FONT_SCALE; ++col)
            {
                int16_t fontPixel = font[ offset + (row*FONT_SIZE + (col / FONT_SCALE))  ];
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
    lcdReset();
    SPI1->CR1 |= SPI_CR1_SPE;
    
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

void ili9341_set_window(int16_t x, int16_t y, int16_t width, int16_t height)
{
    int16_t clipX = x;
    int16_t clipY = y;
    int16_t clipWidth  = width;
    int16_t clipHeight = height;
    // clipping
    if (x+width > LCD_WIDTH)
    {
        clipWidth = width - ((x + width) - LCD_WIDTH);
    }
    else if (x < 0)
    {
        if (x+width < 0) return;
        clipX = 0;
        clipWidth = width - (-x);
    }
    if (y+height > LCD_HEIGHT)
    {
        clipHeight = height - ((y + height) - LCD_HEIGHT);
    }
    else if (y < 0)
    {
        if (y+height < 0) return;
        clipY = 0;
        clipHeight = height - (-y);
    }
    
    //ADDRESS COLUMN
    SPI1_writeCmd(0x2A);
    
    //XDATA
    SPI1_writeData(clipX>>8);
    SPI1_writeData(clipX);
    SPI1_writeData((clipX+clipWidth)>>8);
    SPI1_writeData(clipX+clipWidth);
    
    //ADDRESS PAGE
    SPI1_writeCmd(0x2B);
    
    //YDATA
    SPI1_writeData(clipY>>8);
    SPI1_writeData(clipY);
    SPI1_writeData((clipY+clipHeight)>>8);
    SPI1_writeData(clipY+clipHeight);
    
    //ADDRESS memset
    SPI1_writeCmd(0x2C);
}

void ili9341_draw_pixel(int16_t x, int16_t y, int16_t color)
{
    ili9341_fill_rect(x, y, 1, 1, color);
}

void ili3941_fillscreen(int16_t color)
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

void ili9341_fill_rect(int16_t x, int16_t y, int16_t width, int16_t height, int16_t color)
{
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
    
    // ADC clk prescaler (devide by 6; max 14Mhz!)
    RCC->CFGR |= RCC_CFGR_ADCPRE_1;
    
    RCC->CFGR &= ~RCC_CFGR_SW; // select PLL as SYSCLK
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    
}

static void ADC1_init(void)
{
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_EXTTRIG;
    ADC1->CR2 |= ADC_CR2_EXTSEL;
    ADC1->CR2 &= ~ADC_CR2_ALIGN;
    ADC1->CR2 |= ADC_CR2_ADON;
    HAL_Delay(100);
    
    // calibration
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    ADC1->CR2 |= ADC_CR2_CAL;
}

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

static void MX_GPIO_Init(void)
{
    // CMSIS INIT
    // clock enable for SPI1, GPIOA, AF and ADC
    RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN|RCC_APB2ENR_IOPAEN|RCC_APB2ENR_SPI1EN|RCC_APB2ENR_ADC1EN);
    
    // GPIO DC
    GPIOA->CRL |= GPIO_CRL_MODE0_1;
    GPIOA->CRL &= ~( GPIO_CRL_MODE0_0|GPIO_CRL_CNF0 );
    // GPIO RST
    GPIOA->CRL |= GPIO_CRL_MODE1_1;
    GPIOA->CRL &= ~( GPIO_CRL_MODE1_0|GPIO_CRL_CNF1 );
    // GPIO CS
    GPIOA->CRL |= GPIO_CRL_MODE2_1;
    GPIOA->CRL &= ~( GPIO_CRL_MODE2_0|GPIO_CRL_CNF2 );
    
    // joystick X
    GPIOA->CRL &= ~GPIO_CRL_MODE3;
    GPIOA->CRL &= ~GPIO_CRL_CNF3;
    // joystick Y
    GPIOA->CRL &= ~GPIO_CRL_MODE4;
    GPIOA->CRL &= ~GPIO_CRL_CNF4;
    
    // Push Button Switch
    GPIOA->CRH &= ~GPIO_CRH_MODE8;
    GPIOA->CRH |= GPIO_CRH_CNF8_0;
    
    // SPI1 SCK, MOSI
    GPIOA->CRL |= (GPIO_CRL_MODE5|GPIO_CRL_MODE7); // both to output PP
    GPIOA->CRL |= (GPIO_CRL_CNF5_1|GPIO_CRL_CNF7_1);
    GPIOA->CRL &= ~( GPIO_CRL_CNF5_0|GPIO_CRL_CNF7_0 );
    // SPI1 MISO
    GPIOA->CRL &= ~( GPIO_CRL_MODE6|GPIO_CRL_CNF6_1 );
    GPIOA->CRL |= GPIO_CRL_CNF6_0;
}

// HAL-code from STM

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
