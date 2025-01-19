#include "config.h"
#include "st7735.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "mono9.h"
#include <math.h>
#include <debug.h>

#include "gattprofile.h"
#include "peripheral.h"
#include "app_uart.h"
#include "wchble.h"
#include "BLEHAL.h"


/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
uint8_t const MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
#endif


/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   Main loop
 *
 * @return  none
 */
__attribute__((section(".highcode")))
__attribute__((noinline))
void Main_Circulation(void)
{
    while(1)
    {
        TMOS_SystemProcess();
        app_uart_process();
    }
}
const unsigned char MLX90640_address = 0x66;

#define TA_SHIFT 8 // Default shift for MLX90640 in open air

static float mlx90640To[768];
uint16_t bmp[128 * 64];
float thermalMap[86 * 64];
uint8_t rgbMap[32 * 24 * 3];
paramsMLX90640 mlx90640;

ST7735 lcd = ST7735();

void drawString(const char *str, int x, int y, uint16_t color);
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void loop();

volatile int keyStatus = 0;
void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        keyStatus = 1 - keyStatus;
        EXTI_ClearITPendingBit(EXTI_Line7); /* Clear Flag */
    }
}
void keyInit()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    // pinMode(GPIOC, GPIO_Pin_7, OUTPUT);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    /* GPIOC ----> EXTI_Line7 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    SetVTFIRQ((uint32_t)EXTI9_5_IRQHandler, EXTI9_5_IRQn, 0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
int main(void)
{
    hal_init();
    BLE_HAL_Init();
    WCHBLE_Init();
    GAPRole_PeripheralInit();
    Peripheral_Init();
    app_uart_init();
    Main_Circulation();

    keyInit();
    uint16_t eeMLX90640[832];
    MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    // MLX90640_SetRefreshRate(MLX90640_address, 0x02); // Set rate to 2Hz
    //  MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 4Hz
    MLX90640_SetRefreshRate(MLX90640_address, 0x4); // Set rate to 6Hz

    lcd.begin();
    lcd.clearScreen();
    loop();
    lcd.setBrightness(1);
    while (1)
    {
        loop();
    }
    return 0;
}

uint16_t getColor(float val, float minTemp, float maxTemp)
{
    float a = minTemp + (maxTemp - minTemp) * 0.2121;
    float b = minTemp + (maxTemp - minTemp) * 0.3182;
    float c = minTemp + (maxTemp - minTemp) * 0.4242;
    float d = minTemp + (maxTemp - minTemp) * 0.8182;
    int red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255), green = 0, blue = 0;

    if ((val > minTemp) & (val < a))
    {
        green = constrain(255.0 / (a - minTemp) * val - (255.0 * minTemp) / (a - minTemp), 0, 255);
    }
    else if ((val >= a) & (val <= c))
    {
        green = 255;
    }
    else if (val > c)
    {
        green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
    }
    else if ((val > d) | (val < a))
    {
        green = 0;
    }

    if (val <= b)
    {
        blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
    }
    else if ((val > b) & (val <= d))
    {
        blue = 0;
    }
    else if (val > d)
    {
        blue = constrain(240.0 / (maxTemp - d) * val - (d * 240.0) / (maxTemp - d), 0, 240);
    }
    // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
    return RGBto565(blue, green, red);
}
void bilinearInterpolation(float *src, float *dst, int width, int height, int newWidth, int newHeight);
void loop()
{
    if (keyStatus == 1)
    {
        return;
    }
    memset(bmp, 0, 128 * 64 * 2);
    for (int x = 0; x < 2; x++) // Read both subpages
    {
        uint16_t mlx90640Frame[834];
        MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
        float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    }
    float minT = 200, maxT = -100;
    for (int i = 0; i < 768; i++)
    {
        float T = mlx90640To[i];
        if (T < minT)
            minT = T;
        if (T > maxT)
            maxT = T;
    }
    float wT = 255.0 / (maxT - minT);
    // int maxTI = maxT*255.0/300.0;
    // for (int i = 0; i < 768; i++)
    // {
    //     int x = i % 32;
    //     int y = i / 32;
    //     float g = mlx90640To[i];
    //     getColor(g, minT, maxT, rgbMap + i * 3);
    // }
    bilinearInterpolation(mlx90640To, thermalMap, 32, 24, 64, 48);

    for (int y = 0; y < 48; y++)
    {
        for (int x = 0; x < 64; x++)
        {
            int idx = y+39 + x * 128;
            int idx2 = 3071-(x + y * 64);
            bmp[idx] = getColor(thermalMap[idx2], minT, maxT);
        }
    }
    // for (int y = 0; y < 64; y++)
    // {
    //     for (int x = 0; x < 48; x++)
    //     {
    //         if (!(y % 2 == 0 && x % 2 == 0))
    //         {
    //             if (y % 2 == 0)
    //             {
    //                 int idx = 88 - (x) + (63 - y) * 128;
    //                 int i0 = (y >> 1) + (x >> 1) * 32;
    //                 int i1 = (y >> 1) + 1 + (x >> 1) * 32;
    //                 float g = (mlx90640To[i0] + mlx90640To[i1]) / 2;
    //                 bmp[idx] = getColor(g, minT, maxT);
    //             }
    //             else if (x % 2 == 0)
    //             {
    //                 int idx = 88 - (x) + (63 - y) * 128;
    //                 int i0 = (y >> 1) + (x >> 1) * 32;
    //                 int i1 = (y >> 1) + ((x >> 1) + 1) * 32;
    //                 float g = (mlx90640To[i0] + mlx90640To[i1]) / 2;
    //                 bmp[idx] = getColor(g, minT, maxT);
    //             }
    //             else
    //             {
    //                 int idx = 88 - (x) + (63 - y) * 128;
    //                 int i0 = (y >> 1) + (x >> 1) * 32;
    //                 int i1 = (y >> 1) + 1 + ((x >> 1) + 1) * 32;
    //                 int i2 = (y >> 1) + ((x >> 1) + 1) * 32;
    //                 int i3 = (y >> 1) + 1 + ((x >> 1)) * 32;
    //                 float g = (mlx90640To[i0] + mlx90640To[i1] + mlx90640To[i2] + mlx90640To[i3]) / 4;
    //                 bmp[idx] = getColor(g, minT, maxT);
    //             }
    //         }
    //     }
    // }
    char *temp = (char *)malloc(12);
    memset(temp, 0, 12);
    sprintf(temp, "%.1f", maxT);
    drawString((const char *)temp, 0, 12, RGBto565(0x0, 0x0, 0xff));
    memset(temp, 0, 12);
    sprintf(temp, "%.1f", minT);
    drawString((const char *)temp, 0, 60, RGBto565(0xff, 0x0, 0x0));
    memset(temp, 0, 12);
    sprintf(temp, "%.0f%%", GetBatPercent());
    drawString((const char *)temp, 90, 12, RGBto565(0x0, 0xff, 0x0));
    lcd.drawImage(0, 0, 128, 64, bmp);
    free(temp);
    // Delay_Ms(100);
}
void writePixel(int x, int y, uint16_t color)
{
    int idx = (127 - x) + (63 - y) * 128;
    if (idx >= 0 && idx < 128 * 64)
    {
        bmp[idx] = color;
    }
}
void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    for (int i = 0; i < w; i++)
    {
        for (int j = 0; j < h; j++)
        {
            writePixel(x + i, y + j, color);
        }
    }
}
void drawString(const char *str, int x, int y, uint16_t color)
{
    int i = 0;
    while (str[i])
    {
        drawChar(x + i * 9, y, str[i], color, 0x0, 1);
        i++;
    }
}
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
    c -= (uint8_t)pgm_read_byte(&FreeMono9pt7b.first);
    GFXglyph *glyph = FreeMono9pt7b.glyph + c;
    uint8_t *bitmap = FreeMono9pt7b.bitmap;

    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
    int8_t xo = pgm_read_byte(&glyph->xOffset),
           yo = pgm_read_byte(&glyph->yOffset);
    uint8_t xx, yy, bits = 0, bit = 0;
    int16_t xo16 = 0, yo16 = 0;

    if (size > 1)
    {
        xo16 = xo;
        yo16 = yo;
    }

    for (yy = 0; yy < h; yy++)
    {
        for (xx = 0; xx < w; xx++)
        {
            if (!(bit++ & 7))
            {
                bits = pgm_read_byte(&bitmap[bo++]);
            }
            if (bits & 0x80)
            {
                if (size == 1 && size == 1)
                {
                    writePixel(x + xo + xx, y + yo + yy, color);
                }
                else
                {
                    writeFillRect(x + (xo16 + xx) * size, y + (yo16 + yy) * size,
                                  size, size, color);
                }
            }
            bits <<= 1;
        }
    }
}

float interpolate(float k, float kMin, float kMax, float vMin, float vMax)
{
    return roundf((k - kMin) * vMax + (kMax - k) * vMin);
}
float interpolateHorizontal(float *src, int width, int x, int y, int xMin, int xMax)
{
    float vMin = src[(y * width + xMin)];
    if (xMin == xMax)
        return vMin;
    float vMax = src[(y * width + xMax)];
    return interpolate(x, xMin, xMax, vMin, vMax);
}
float interpolateVertical(float *src, int width, int x, int xMin, int xMax, int y, int yMin, int yMax)
{
    float vMin = interpolateHorizontal(src, width, x, yMin, xMin, xMax);
    if (yMin == yMax)
        return vMin;
    float vMax = interpolateHorizontal(src, width, x, yMax, xMin, xMax);
    return interpolate(y, yMin, yMax, vMin, vMax);
}
void bilinearInterpolation(float *src, float *dst, int width, int height, int newWidth, int newHeight)
{
    int pos = 0;

    for (float y = 0; y < newHeight; y++)
    {
        for (float x = 0; x < newWidth; x++)
        {
            float srcX = x * width / newWidth;
            float srcY = y * height / newHeight;

            float xMin = floorf(srcX);
            float yMin = floorf(srcY);

            float xMax = fmin(ceilf(srcX), width - 1);
            float yMax = fmin(ceilf(srcY), height - 1);

            dst[pos++] = interpolateVertical(src, width, srcX, xMin, xMax, srcY, yMin, yMax); // R
        }
    }
}