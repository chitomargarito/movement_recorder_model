/**
  ******************************************************************************
  * @file    BSP/Inc/mems.h 
  * @author  MCD Application Team
  * @brief   Header for mems.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEMS_H
#define __MEMS_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"


#define READWRITE_CMD ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD ((uint8_t)0x40)
#define CS_GPIO_PORT GPIOE
#define CS_PIN GPIO_PIN_3
#define CS_ON HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET)
#define CS_OFF HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET)
#define DUMMY_BYTE ((uint8_t)0x00)

#define LIS3DSH_CTRL_REG4_ADDR               0x20
#define LIS3DSH_CTRL_REG5_ADDR               0x24

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

//LED Indication
void Error(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

void Success(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

//I2C Operations

static uint8_t I2C_Read(uint16_t Addr, uint8_t Reg)

{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 0x10000);
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    Error();
  }
  else Success();
  return value;
}

static void I2C_Write(uint16_t Addr, uint8_t Reg, uint8_t Value)

{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 0x10000);
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    Error();
  }
  else Success();
}

uint8_t I2C_ReadID(uint16_t Addr)
{
    uint8_t ctrl = 0x00;
    ctrl = I2C_Read(0x32, 0x0F);
    return ctrl;
}



//Accelerometer Operations

void Accel_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value)
{
    I2C_Write(DeviceAddr, RegisterAddr, Value);
}

void Accel_Ini(void)
{
  uint8_t ctrl;
  if (I2C_ReadID(0x32)==0x33)
  {
      ctrl = 0b01100111;
      I2C_Write(0x32,0x20,ctrl);
      ctrl = 0b00001000;
      I2C_Write(0x32,0x23,ctrl);
  }
  HAL_Delay(500);
}

void Accel_GetXYZ(int16_t* pData)

{
  int8_t buffer[6];
  uint8_t i = 0;

  buffer[0] = I2C_Read(0x32, 0x28);
  buffer[1] = I2C_Read(0x32, 0x29);
  buffer[2] = I2C_Read(0x32, 0x2A);
  buffer[3] = I2C_Read(0x32, 0x2B);
  buffer[4] = I2C_Read(0x32, 0X2C);
  buffer[5] = I2C_Read(0x32, 0x2D);

  for(i=0; i<3; i++)
  {
    pData[i]=((int16_t)((uint16_t)buffer[2*i+1]<<8)+buffer[2*i]);
  }
}

//Magnetometer operation
void Mag_Ini(void)
{
  uint8_t ctrl;
  if (I2C_ReadID(0x3C)==0x3C)
  {
      ctrl = 0b00011100;
      I2C_Write(0x3C,0x00,ctrl);
      ctrl = 0b00100000;
      I2C_Write(0x3C,0x01,ctrl);
      ctrl = 0b00000000;
      I2C_Write(0x3C,0x02,ctrl);
  }
  HAL_Delay(500);
}


void Mag_GetXYZ(int16_t* pData)
{
  int8_t buffer[6];
  uint8_t i = 0;

  buffer[0] = I2C_Read(0x3C, 0x03);
  buffer[1] = I2C_Read(0x3C, 0x04);
  buffer[2] = I2C_Read(0x3C, 0x05);
  buffer[3] = I2C_Read(0x3C, 0x06);
  buffer[4] = I2C_Read(0x3C, 0X07);
  buffer[5] = I2C_Read(0x3C, 0x08);

  for(int i=0; i<3; i++)
  {
    pData[i]=((int16_t)((uint16_t)buffer[2*i]<<8)+buffer[2*i+1]);
  }
}

//SPI Operations
static uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;
  if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, 0x1000) != HAL_OK)
  {
      Error();
  }
  else
  {
      Success();
  }
  return receivedbyte;
}

//Gyroscope Operation

void Gyro_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
        CS_ON;
  SPIx_WriteRead(ReadAddr);
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to ACCELEROMETER (Slave device) */
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
        CS_OFF;
}

void Gyro_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
        CS_OFF;
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
        CS_ON;
        SPIx_WriteRead(WriteAddr);
        while(NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
        CS_OFF;
}

uint8_t Gyro_ReadID(void)
{
  uint8_t ctrl = 0;
  Gyro_IO_Read(&ctrl, 0x0F, 1);
  return ctrl;
}

void Gyro_Ini(void)
{
  uint8_t ctrl = 0x00;
  if (Gyro_ReadID()==0xD4)
  {
      ctrl = 0b11111111;
      Gyro_IO_Write(&ctrl,0x20,1);
      ctrl = 0b00100000;
      Gyro_IO_Write(&ctrl,0x21,1);
      ctrl = 0b00001000;
      Gyro_IO_Write(&ctrl,0x22,1);
      ctrl = 0b00001000;
      Gyro_IO_Write(&ctrl,0x23,1);
      ctrl = 0b00010000;
      Gyro_IO_Write(&ctrl,0x24,1);
  }
  else Error();
  HAL_Delay(500);
}

void Gyro_GetXYZ(int16_t* pData)
{
  uint8_t buffer[6];
  float valueinfloat = 0;

  Gyro_IO_Write((uint8_t*)&buffer[0], 0x28, 1);
  Gyro_IO_Write((uint8_t*)&buffer[1], 0x29, 1);
  Gyro_IO_Write((uint8_t*)&buffer[2], 0x2A, 1);
  Gyro_IO_Write((uint8_t*)&buffer[3], 0x2B, 1);
  Gyro_IO_Write((uint8_t*)&buffer[4], 0x2C, 1);
  Gyro_IO_Write((uint8_t*)&buffer[5], 0x2D, 1);

  for(int i=0; i<3; i++)
  {
    valueinfloat = ((buffer[2*i+1] << 8) + buffer[2*i]);
    pData[i] = (int16_t)valueinfloat;
  }
}





/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//void ACCELERO_MEMS_Test(void);
//void GYRO_MEMS_Test(void);
#endif /* __MEMS_H */
