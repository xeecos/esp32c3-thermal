#pragma once
#include <ch32v20x.h>
#define INPUT     GPIO_Mode_IN_FLOATING
#define OUTPUT    GPIO_Mode_Out_PP

#define LOW       0
#define HIGH      1
void hal_init();
void pinMode(GPIO_TypeDef* port, uint16_t pin, GPIOMode_TypeDef mode);
void digitalWrite(GPIO_TypeDef* port, uint16_t pin, int val);
int digitalRead(GPIO_TypeDef* port, uint16_t pin);

#define pgm_read_word(addr) (*(const unsigned short *)(addr)) 
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))