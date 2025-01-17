#include "config.h"
#include "st7735.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "mono9.h"
#include <debug.h>
const unsigned char MLX90640_address = 0x66;

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

static float mlx90640To[768];
uint16_t bmp[128*64];
paramsMLX90640 mlx90640;

ST7735 lcd = ST7735();

void drawString(const char*str, int x, int y, uint16_t color);
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void loop();
int main(void)
{

    hal_init();
    int status;
    uint16_t eeMLX90640[832];
    MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

    //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 2Hz
    // MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 4Hz
    MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 64Hz

    lcd.begin();
    lcd.setRotation(2);
    while(1)
    {
        // lcd.setBrightness(0);
        // Delay_Ms(500);
        // lcd.setBrightness(1);
        // Delay_Ms(500);

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
  int red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255), green, blue;

  if ((val > minTemp) & (val < a)) {
    green = constrain(255.0 / (a - minTemp) * val - (255.0 * minTemp) / (a - minTemp), 0, 255);
  }
  else if ((val >= a) & (val <= c)) {
    green = 255;
  }
  else if (val > c) {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a)) {
    green = 0;
  }

  if (val <= b) {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d)) {
    blue = 0;
  }
  else if (val > d) {
    blue = constrain(240.0 / (maxTemp - d) * val - (d * 240.0) / (maxTemp - d), 0, 240);
  }

  // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return RGBto565(blue, green, red);
}
void loop()
{
    memset(bmp,0,128*64*2);
    for (int x = 0 ; x < 2 ; x++) //Read both subpages
    {
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0)
        {
            // USBSerial.print("GetFrame Error: ");
        }
        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;

        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    }
    float minT = 200, maxT = -100;
    for(int i=0;i<768;i++)
    {
        float T = mlx90640To[i];
        if(T<minT) minT = T;
        if(T>maxT) maxT = T;
    }
    float wT = 255.0/(maxT - minT);
    // int maxTI = maxT*255.0/300.0;
    for(int i=0;i<768;i++)
    {
        int x = i % 32;
        int y = i / 32;
        int idx = 88 - (y<<1) + (63-(x<<1)) * 128;
        float g = mlx90640To[i];
        bmp[idx] = getColor(g, minT, maxT);
    }
    for(int y=0;y<64;y++)
    {
        for(int x=0;x<48;x++)
        {
            if(!(y%2==0&&x%2==0))
            {
                if(y%2==0)
                {
                    int idx = 88 - (x) + (63-y) * 128;
                    int i0 = (y>>1) + (x>>1) * 32;
                    int i1 = (y>>1)+1 + (x>>1) * 32;
                    float g = (mlx90640To[i0]+mlx90640To[i1])/2;
                    bmp[idx] = getColor(g, minT, maxT);
                }
                else if(x%2==0)
                {
                    int idx = 88 - (x) + (63-y) * 128;
                    int i0 = (y>>1) + (x>>1) * 32;
                    int i1 = (y>>1) + ((x>>1)+1) * 32;
                    float g = (mlx90640To[i0]+mlx90640To[i1])/2;
                    bmp[idx] = getColor(g, minT, maxT);
                }
                else
                {
                    int idx = 88 - (x) + (63-y) * 128;
                    int i0 = (y>>1) + (x>>1) * 32;
                    int i1 = (y>>1)+1 + ((x>>1)+1) * 32;
                    int i2 = (y>>1) + ((x>>1)+1) * 32;
                    int i3 = (y>>1)+1 + ((x>>1)) * 32;
                    float g = (mlx90640To[i0]+mlx90640To[i1]+mlx90640To[i2]+mlx90640To[i3])/4;
                    bmp[idx] = getColor(g, minT, maxT);
                }
            }
        }
    }
    char* temp = (char*)malloc(12);
    memset(temp,0,12);
    sprintf(temp, "%.1f", maxT);
    drawString((const char*)temp,0,12,RGBto565(0x0, 0x0, 0xff));
    memset(temp,0,12);
    sprintf(temp, "%.1f", minT);
    drawString((const char*)temp,0,60,RGBto565(0xff, 0x0, 0x0));
    lcd.drawImage(0,0,128,64,bmp);
    free(temp);
    // Delay_Ms(100);
}
void writePixel(int x, int y, uint16_t color)
{
    int idx = (127-x) + (63-y) * 128;
    if(idx>=0&&idx<128*64)
    {
        bmp[idx] = color;
    }
}
void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    for(int i=0;i<w;i++)
    {
        for(int j=0;j<h;j++)
        {
            writePixel(x+i,y+j,color);
        }
    }
}
void drawString(const char*str, int x, int y, uint16_t color)
{
    int i = 0;
    while (str[i])
    {
        drawChar(x+i*9,y,str[i],color,0x0,1);
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

    for (yy = 0; yy < h; yy++) {
        for (xx = 0; xx < w; xx++) {
            if (!(bit++ & 7)) {
            bits = pgm_read_byte(&bitmap[bo++]);
            }
            if (bits & 0x80) {
            if (size == 1 && size == 1) {
                writePixel(x + xo + xx, y + yo + yy, color);
            } else {
                writeFillRect(x + (xo16 + xx) * size, y + (yo16 + yy) * size,
                            size, size, color);
            }
            }
            bits <<= 1;
        }
    }
}