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

#include "MLX90640_I2C_Driver.h"
#include <ch32v20x.h>
/**
 * @brief  Initializes the I2C peripheral used to drive the HMC5883L
 * @param  None
 * @retval None
 */
void Wire_Init()
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(HAL_I2C_RCC_Periph, ENABLE);
    RCC_APB2PeriphClockCmd(HAL_I2C_RCC_Port, ENABLE);

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = HAL_I2C_SCL_Pin | HAL_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(HAL_I2C_Port, &GPIO_InitStructure);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x02;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = HAL_I2C_Speed;

    /* Apply I2C configuration after enabling it */
    I2C_Init(HAL_I2C, &I2C_InitStructure);

    I2C_Cmd(HAL_I2C, ENABLE);
}

/**
 * @brief  Writes one byte to the  HMC5883L.
 * @param  slaveAddr : slave address HAL_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the HMC5883L.
 * @param  WriteAddr : address of the register in which the data will be written
 * @retval None
 */
void Wire_ByteWrite(uint8_t slaveAddr, uint16_t pBuffer, uint8_t WriteAddr)
{
    // ENTR_CRT_SECTION();

    /* Send START condition */
    I2C_GenerateSTART(HAL_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_MODE_SELECT))
        ;

    /* Send HMC5883 address for write */
    I2C_Send7bitAddress(HAL_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;

    /* Send the HMC5883L internal address to write to */
    I2C_SendData(HAL_I2C, WriteAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;

    /* Send the byte to be written */
    I2C_SendData(HAL_I2C, pBuffer>>8);
    I2C_SendData(HAL_I2C, pBuffer&0xff);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;

    /* Send STOP condition */
    I2C_GenerateSTOP(HAL_I2C, ENABLE);
    // EXT_CRT_SECTION();
}

/**
 * @brief  Reads a block of data from the HMC5883L.
 * @param  slaveAddr  : slave address HAL_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the HMC5883L.
 * @param  ReadAddr : HMC5883L's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the HMC5883L ( NumByteToRead >1  only for the Magnetometer reading).
 * @retval None
 */
void Wire_BufferRead(uint8_t slaveAddr, uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    // ENTR_CRT_SECTION();

    /* While the bus is busy */
    while (I2C_GetFlagStatus(HAL_I2C, I2C_FLAG_BUSY))
        ;

    /* Send START condition */
    I2C_GenerateSTART(HAL_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_MODE_SELECT))
        ;

    /* Send HAL_Magn address for write */ // Send HMC5883L address for write
    I2C_Send7bitAddress(HAL_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ;

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(HAL_I2C, ENABLE);

    /* Send the HMC5883L's internal address to write to */
    I2C_SendData(HAL_I2C, ReadAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ;

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(HAL_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_MODE_SELECT))
        ;

    /* Send HMC5883L address for read */
    I2C_Send7bitAddress(HAL_I2C, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
        ;

    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(HAL_I2C, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(HAL_I2C, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(HAL_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the HMC5883L */
            *pBuffer = I2C_ReceiveData(HAL_I2C);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(HAL_I2C, ENABLE);
    // EXT_CRT_SECTION();
}
void MLX90640_I2CInit()
{
    Wire_Init();
}

// Read a number of words from startAddress. Store into Data array.
// Returns 0 if successful, -1 if error
int MLX90640_I2CRead(uint8_t _deviceAddress, unsigned int startAddress, unsigned int nWordsRead, uint16_t *data)
{
    int i = 0;

    uint8_t i2cData[1664] = {0};
    uint16_t *p;
    uint8_t *p8 = &i2cData[0];
    p = data;

    Wire_BufferRead(_deviceAddress, p8, startAddress, nWordsRead*2);

    for(int cnt=0; cnt < nWordsRead; cnt++)
    {
        i = cnt << 1;
        *p++ = (uint16_t)i2cData[i]*256 + (uint16_t)i2cData[i+1];
    }
    
    return 0; 
}

// Set I2C Freq, in kHz
// MLX90640_I2CFreqSet(1000) sets frequency to 1MHz
void MLX90640_I2CFreqSet(int freq)
{
    // i2c.frequency(1000 * freq);
}

// Write two bytes to a two byte address
int MLX90640_I2CWrite(uint8_t _deviceAddress, unsigned int writeAddress, uint16_t data)
{
    uint8_t cmd[2] = {0};
    static uint16_t dataCheck;


    cmd[0] = data >> 8;
    cmd[1] = data & 0x00FF;

    uint8_t *p = &cmd[0];
    Wire_ByteWrite(_deviceAddress, data, writeAddress); 
    
    MLX90640_I2CRead(_deviceAddress,writeAddress,1, &dataCheck);
    
    if ( dataCheck != data)
    {
        return -2;
    }    
    
    return 0;
}
