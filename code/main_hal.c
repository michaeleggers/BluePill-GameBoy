
#include "main.h"
#include <stdlib.h>

#include "ili9341.h"
#include "font.h"

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
    //int16_t xPosOld, yPosOld;
    int16_t width;
    int16_t height;
    int16_t scale;
    //int16_t xVel; // NOTE(Michael): move later into Entity_t or something like that
    //int16_t yVel;
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

typedef struct Entity_t
{
    float xPos, yPos;
    float xPosOld, yPosOld;
    Bitmap_t *bitmap;
} Entity_t;

typedef struct Pixelpos_t
{
    int16_t x, y;
} Pixelpos_t;

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
void ili9341_draw_bitmap(Entity_t * entity);
void draw_static_bitmap(Bitmap_t * bitmap, int16_t xPos, int16_t yPos, int16_t scale);
void showTitleScreen();
void showScoreScreen();
void initGameAssets();
void gameLoop();
void lcdPrint(int16_t x, int16_t y, char const * string, uint8_t length, uint8_t scale, uint16_t colorFG, uint16_t colorBG);

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

// globals
#define PIPE_PAIR_COUNT 1
Pipepair_t pipePairs[PIPE_PAIR_COUNT];
Bitmap_t stone_floor = {
    0, 0,
    //0, 0,
    16, 16,
    1,
    //0, 0,
    gImage_stone_floor,
    BMP_RGB565_8
};

Bitmap_t box = {
    30, 120,
    //0, 0,
    8, 8,
    4,
    //0, 0,
    gImage_box,
    BMP_RGB565_16
};

Bitmap_t titleScreenBmp = {
    0, 0,
    16, 16,
    1,
    &tanya,
    BMP_RGB565_16
};

Bitmap_t scoreScreenBmp = {
    0, 0,
    16, 16,
    1,
    &tanya,
    BMP_RGB565_16
};

Entity_t player = {
    30, 120,
    30, 120,
    &box
};

#define AIRBUBBLE_COUNT 5
Pixelpos_t gAirBubbles[AIRBUBBLE_COUNT];

static uint16_t gScore;
static uint8_t gUiUpdateFlag;

void showTitleScreen()
{
    //draw_static_bitmap(&titleScreenBmp, 0, 0, 10);
    ili3941_fillscreen(ILI9341_BLUE);
    lcdPrint(0, 0, "press to start", 14, 2, ILI9341_WHITE, ILI9341_BLUE);
    uint8_t buttonPressed = 0;
    while ( !buttonPressed )
    {
        buttonPressed = (GPIOA->IDR & GPIO_IDR_IDR8) == GPIO_IDR_IDR8;
    }
    ili3941_fillscreen(ILI9341_BLUE);
}

void showScoreScreen()
{
    ili3941_fillscreen(ILI9341_BLUE);
    lcdPrint(0, 0, "Score", 5, 4, ILI9341_WHITE, ILI9341_BLUE);
    char score[64];
    itoa(gScore, score, 10);
    lcdPrint(0, 4*8+8, score, 1, 4, ILI9341_WHITE, ILI9341_BLUE);
    //draw_static_bitmap(&scoreScreenBmp, 0, 0, 10);
    HAL_Delay(500);
    uint8_t buttonPressed = 0;
    while ( !buttonPressed )
    {
        buttonPressed = (GPIOA->IDR & GPIO_IDR_IDR8) == GPIO_IDR_IDR8;
    }
    ili3941_fillscreen(ILI9341_BLUE);
}

#define PIPE_WIDTH 40
void initGameAssets()
{
    Pipe_t pipe1 = { 
        LCD_WIDTH, 16, // upper UI element is 16 pixels high
        PIPE_WIDTH, 100-16,
        LCD_WIDTH
    };
    Pipe_t pipe2 = { 
        LCD_WIDTH, 210,
        PIPE_WIDTH, 120,
        LCD_WIDTH
    };
    Pipepair_t pipePair = {
        pipe1,
        pipe2,
    };
    pipePairs[0] = pipePair;
    for (int i = 0; i < AIRBUBBLE_COUNT; ++i)
    {
        gAirBubbles[i].x = ((float)rand()/(float)RAND_MAX) * LCD_WIDTH;
        gAirBubbles[i].y = 16 + ((float)rand()/(float)RAND_MAX) * LCD_HEIGHT;
    }
    gScore = 0;
    gUiUpdateFlag = 1;
}

void gameLoop()
{
    ili9341_fill_rect(0, 0, LCD_WIDTH, 16, ILI9341_BLACK);
    lcdPrint(0, 0, "SCORE", 5, 2, ILI9341_WHITE, ILI9341_BLACK);
    float vertSpeed = 0;
    float fallingConstant = 0.2f;
    float jumpConstant = -3;
    volatile int isGameOver = 0;
    while (!isGameOver) // main game loop
    {
        uint8_t buttonPressed = (GPIOA->IDR & GPIO_IDR_IDR8) == GPIO_IDR_IDR8;
        if (buttonPressed)
        {
            //player.yPos -= 5;
            vertSpeed = jumpConstant;
            player.bitmap = &box;
        }
        else
        {
            player.bitmap = &box;
        }
        player.yPos += vertSpeed;
        vertSpeed += fallingConstant;
        
        // check screen boundaries
        if (player.yPos <= 16) // UI height is 16 pixels
        {
            player.yPos = 16;
        }
        else if (player.yPos+player.bitmap->height*player.bitmap->scale > LCD_HEIGHT)
        {
            player.yPos = player.yPos - ((player.yPos+player.bitmap->height*player.bitmap->scale) - LCD_HEIGHT);
        }
        
        // collision detection with pipes
        if ( (player.xPos + player.bitmap->scale*player.bitmap->width > pipePairs[0].upper.xPos &&
              player.xPos + player.bitmap->scale*player.bitmap->width < pipePairs[0].upper.xPos + PIPE_WIDTH) ||
            (player.xPos > pipePairs[0].upper.xPos &&
             player.xPos < pipePairs[0].upper.xPos + PIPE_WIDTH) ) // pipe width is 40 pixels
        {
            // check if player hits upper or lower pipe
            if ( (player.yPos < pipePairs[0].upper.height+pipePairs[0].upper.yPos) ||
                (player.yPos + player.bitmap->scale*player.bitmap->height > pipePairs[0].lower.yPos) ) // COLLISION!
            {
                player.bitmap = &stone_floor;
                isGameOver = 1;
            }
        }
        ili9341_draw_bitmap(&player);
        
        // if player made it through, update score ui
        if ( gUiUpdateFlag && (player.xPos > pipePairs[0].upper.xPos + PIPE_WIDTH) )
        {
            gUiUpdateFlag = 0; // already updated
            gScore += 1;
            char score[64];
            itoa(gScore, score, 10);
            lcdPrint(5*2*8+8, 0, score, 1, 2, ILI9341_WHITE, ILI9341_BLACK);
        }
        
        for (int i = 0; i < PIPE_PAIR_COUNT; ++i)
        {
            Pipepair_t *pipePair = &pipePairs[i];
            Pipe_t *upperPipe = &pipePair->upper;
            Pipe_t *lowerPipe = &pipePair->lower;
            // upperPipe
            upperPipe->xPosOld = upperPipe->xPos;
            upperPipe->xPos -= 1;
            int16_t xClear = upperPipe->xPos + upperPipe->width;
            int16_t widthClear = upperPipe->xPosOld - upperPipe->xPos;
            ili9341_fill_rect(xClear, upperPipe->yPos, widthClear, upperPipe->height, ILI9341_BLUE);
            ili9341_fill_rect(upperPipe->xPos, upperPipe->yPos, upperPipe->width, upperPipe->height, ILI9341_GREEN);
            // lowerPipe
            lowerPipe->xPosOld = lowerPipe->xPos;
            lowerPipe->xPos -= 1;
            int16_t xClearLower = lowerPipe->xPos + lowerPipe->width;
            int16_t widthClearLower = lowerPipe->xPosOld - lowerPipe->xPos;
            ili9341_fill_rect(xClearLower, lowerPipe->yPos, widthClearLower, lowerPipe->height, ILI9341_BLUE);
            ili9341_fill_rect(lowerPipe->xPos, lowerPipe->yPos, lowerPipe->width, lowerPipe->height, ILI9341_GREEN);
            if ( (upperPipe->xPos+upperPipe->width) <= 0 ) // pipes moved left OOS
            {
                gUiUpdateFlag = 1;
                ili9341_fill_rect(0, upperPipe->yPos, 1, upperPipe->height, ILI9341_BLUE);
                upperPipe->xPos = LCD_WIDTH;
                ili9341_fill_rect(0, lowerPipe->yPos, 1, lowerPipe->height, ILI9341_BLUE);
                lowerPipe->xPos = LCD_WIDTH;
                // create new hole
                int16_t holeY = ((float)rand()/(float)RAND_MAX) * 190; // max reach of upper pipe before whole starts
                upperPipe->height = holeY;
                lowerPipe->yPos = holeY+130;
                lowerPipe->height = LCD_HEIGHT - lowerPipe->yPos;
            }
        }
        
        // draw Air Bubbles
        for (int i = 0; i < AIRBUBBLE_COUNT; ++i)
        {
            int16_t *x = &(gAirBubbles[i].x);
            int16_t *y = &(gAirBubbles[i].y);
            if (*x+3 <= 0) *x = LCD_WIDTH;
            ili9341_fill_rect(*x, *y, 3, 3, ILI9341_BLUE);
            *x -= 5;
            ili9341_fill_rect(*x, *y, 3, 3, ILI9341_WHITE);
        }
        
        //HAL_Delay(1);
    }
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
        showTitleScreen();
        initGameAssets();
        gameLoop();
        showScoreScreen();
    }
}

/* bitmap width, height MUST be a multiple of 8 pixels!!! */
void ili9341_draw_bitmap(Entity_t * entity)
{
    Bitmap_t *bitmap = entity->bitmap;
    int16_t xPos = entity->xPos;
    int16_t yPos = entity->yPos;
    int16_t xPosOld = entity->xPosOld;
    int16_t yPosOld = entity->yPosOld;
    int16_t width = bitmap->width;
    int16_t height = bitmap->height;
    int16_t scale = bitmap->scale;
    BitmapByteConfig config = bitmap->config;
    
    /*
    if (yPos <= 0)
    {
    yPos = 0;
    }
    else if (yPos+height > LCD_HEIGHT)
    {
    yPos = yPos - ((yPos+height) - LCD_HEIGHT);
    }
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
    */
    if (yPos > yPosOld)
    {
        int16_t clearHeight = (yPos-yPosOld);
        ili9341_fill_rect(xPosOld, yPosOld, width*scale, clearHeight*scale, ILI9341_BLUE);
    }
    if (yPos < yPosOld)
    {
        int16_t clearHeight = yPosOld - yPos;
        ili9341_fill_rect(xPosOld, yPos+scale*height, width*scale, clearHeight, ILI9341_BLUE);
    }
    if((xPos + width*scale > LCD_WIDTH) || (yPos + height*scale > LCD_HEIGHT))
    {
        entity->xPosOld = xPos;
        entity->yPosOld = yPos;
        return;	//OUT OF BOUNDS!
    }
    
    // draw to device
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
    entity->xPosOld = xPos;
    entity->yPosOld = yPos;
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

// ########

#define FONT_SIZE   6
#define FONT_SCALE  5
#define FONT_WIDTH  6
#define FONT_HEIGHT 8
void lcdPrint(int16_t x, int16_t y, char const * string, uint8_t length, uint8_t scale, uint16_t colorFG, uint16_t colorBG)
{
    if((x+FONT_WIDTH*scale > LCD_WIDTH) || (y+FONT_HEIGHT*scale > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    
    int16_t posOffset = 0;
    while (posOffset < length)
    {
        int16_t fontIndex = string[posOffset] - ' ';
        for (int colScale = 0; colScale < scale; ++colScale)
        {
            for (int col = 0; col < FONT_WIDTH; ++col)
            {
                uint8_t fontColumn = fontWithHeart[fontIndex][col];
                for (int row = 0; row < FONT_HEIGHT; ++row)
                {
                    uint8_t fontPixel = (fontColumn & (0x01 << row));
                    uint16_t pixelColor = colorBG;
                    if (fontPixel)
                    {
                        pixelColor = colorFG;
                    }
                    for (int i = 0; i < scale; ++i)
                    {
                        //ili9341_draw_pixel(x + posOffset*FONT_WIDTH + col, y+row*scale+i, pixelColor);
                        ili9341_draw_pixel(x + posOffset*FONT_WIDTH*scale + scale*col+colScale, y+row*scale+i, pixelColor);
                    }
                }
            }
        }
        posOffset++;
    }
    
    /* Backup
    if((x+FONT_WIDTH*scale > LCD_WIDTH) || (y+FONT_HEIGHT*scale > LCD_HEIGHT)) return;	//OUT OF BOUNDS!
    
    int16_t posOffset = 0;
    while (posOffset < length)
    {
        int16_t fontIndex = string[posOffset] - ' ';
        
        for (int col = 0; col < FONT_WIDTH; ++col)
        {
            uint8_t fontColumn = fontWithHeart[fontIndex][col];
            for (int row = 0; row < FONT_HEIGHT; ++row) // each column is a byte (so Font-height = 8)
            {
                uint8_t fontPixel = (fontColumn & (0x01 << row));
                for (int i = 0; i < scale; ++i)
                {
                    if (fontPixel)
                    {
                        ili9341_draw_pixel(x+posOffset*FONT_WIDTH+col, y+row*i, color);
                    }
                    else
                    {
                        ili9341_draw_pixel(x+posOffset*FONT_WIDTH+col, y+row*i, 0x00);
                    }
                }
            }
        }
        posOffset++;
    }
    */
}

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
    ILI9341_Set_Rotation(SCREEN_VERTICAL_2);
    
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
