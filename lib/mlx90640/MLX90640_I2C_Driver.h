/**
   @copyright (C) 2017 Melexis N.V.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

*/
#ifndef _MLX90640_I2C_Driver_H_
#define _MLX90640_I2C_Driver_H_

#include <stdint.h>

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#elif __MK20DX256__
//Teensy 3.2
#define I2C_BUFFER_LENGTH 32

#else

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define HAL_I2C                  I2C1
#define HAL_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define HAL_I2C_Port             GPIOB
#define HAL_I2C_SCL_Pin          GPIO_Pin_6
#define HAL_I2C_SDA_Pin          GPIO_Pin_7
#define HAL_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define HAL_I2C_Speed            100000

void Wire_Init();
void Wire_ByteWrite(uint8_t  slaveAddr, uint16_t pBuffer, uint8_t  WriteAddr);
void Wire_BufferRead(uint8_t  slaveAddr,uint8_t * pBuffer, uint8_t  ReadAddr, uint16_t  NumByteToRead);

void MLX90640_I2CInit(void);
int MLX90640_I2CRead(uint8_t slaveAddr, unsigned int startAddress, unsigned int nWordsRead, uint16_t *data);
int MLX90640_I2CWrite(uint8_t slaveAddr, unsigned int writeAddress, uint16_t data);
void MLX90640_I2CFreqSet(int freq);
#endif
