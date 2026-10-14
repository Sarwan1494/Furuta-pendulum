#include "as5600.h"

uint16_t AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2];
    uint16_t angle;

    // Read 2 bytes starting from RAW ANGLE HIGH register
    HAL_I2C_Mem_Read(
        hi2c,
        AS5600_I2C_ADDR,
        AS5600_RAW_ANGLE_H,
        I2C_MEMADD_SIZE_8BIT,
        data,
        2,
        HAL_MAX_DELAY);

    // Combine bytes
    angle = ((uint16_t)data[0] << 8) | data[1];

    // Keep only 12 bits as per sensor's data
    angle &= 0x0FFF; // 0x0FFF = 0000 1111 1111 1111

    return angle;
}

float AS5600_ReadAngleDeg(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw;
    float angle_deg;

    raw = AS5600_ReadRawAngle(hi2c);

    angle_deg = (raw * 360.0f) / 4096.0f; // Convert the raw values to degress. Since it is 12 bit, 4096 is used

    return angle_deg;
}
