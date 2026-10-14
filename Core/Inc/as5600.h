#ifndef AS5600_H
#define AS5600_H

#include "stm32f4xx_hal.h"

#define AS5600_I2C_ADDR      (0x36 << 1)

#define AS5600_RAW_ANGLE_H   0x0C
#define AS5600_RAW_ANGLE_L   0x0D

uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c);
float AS5600_ReadAngleDeg(I2C_HandleTypeDef *hi2c);

#endif
