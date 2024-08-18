#include <config.h>
#include <st7735.h>
#include "driver/adc.h"
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

static float mlx90640To[768];
uint16_t bmp[128*64];
paramsMLX90640 mlx90640;

ST7735 lcd = ST7735(PIN_DC, PIN_RST, -1);

boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}


void setup(void) 
{

    Wire.begin();
    Wire.setClock(400000); //Increase I2C clock speed to 400kHz
    USBSerial.begin(115200);
    pinMode(PIN_LED, OUTPUT);
    if (isConnected() == false)
    {
        USBSerial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
    }
    USBSerial.println("MLX90640 online!");
    int status;
    uint16_t eeMLX90640[832];
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
    if (status != 0)
        USBSerial.println("Failed to load system parameters");

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0)
        USBSerial.println("Parameter extraction failed");

    //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 2Hz
    MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 4Hz
    //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 64Hz
}
void loop()
{
    for (byte x = 0 ; x < 2 ; x++) //Read both subpages
    {
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0)
        {
            USBSerial.print("GetFrame Error: ");
            USBSerial.println(status);
        }

        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;

        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    }
    for(int i=0;i<768;i++)
    {
        int x = i%32;
        int y = i/32;
        int idx = x+y*128;
        int g = mlx90640To[i]*255;
        bmp[idx] = RGBto565(g,g,g);
    }
    lcd.drawImage(0,0,128,64,bmp);
    delay(30);
}
