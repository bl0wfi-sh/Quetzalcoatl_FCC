/*
 * I2CWrapper.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: heapr0ll
 */

#include "I2CWrapper.hpp"

void I2CWrapper::write8(uint8_t reg, uint8_t val) {
    HAL_I2C_Mem_Write(hi2c, (uint16_t)(dev_add<<1), reg, 1, &val, 1, HAL_MAX_DELAY);
}

uint8_t I2CWrapper::read8(uint8_t reg) {
    uint8_t data = 0;

    HAL_StatusTypeDef code = HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_add<<1), reg, 1, &data, 1, HAL_MAX_DELAY);

    if (code != HAL_OK)
    {
    	printf("[I2CWrapper] - Error %d while trying to read %d bytes from REG %d\r\n", code, 1, reg);
    }

    return data;
}

uint16_t I2CWrapper::read16(uint8_t reg) {
    uint8_t data[2];

    HAL_StatusTypeDef code = HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_add<<1), reg, 1, data, 2, HAL_MAX_DELAY);

    if (code != HAL_OK)
    {
    	printf("[I2CWrapper] - Error %d while trying to read %d bytes from REG %d\r\n", code, 2, reg);
    }

    return (data[0] | (data[1] << 8));
}

uint16_t I2CWrapper::read16Be(uint8_t reg) {
    uint8_t data[2];

    HAL_StatusTypeDef code = HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_add<<1), reg, 1, data, 2, HAL_MAX_DELAY);

    if (code != HAL_OK)
    {
    	printf("[I2CWrapper] - Error %d while trying to read %d bytes from REG %d\r\n", code, 2, reg);
    }

    return (data[1] | (data[0] << 8));
}

uint32_t I2CWrapper::read24(uint8_t reg) {
    uint8_t data[3];

    HAL_StatusTypeDef code = HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_add<<1), reg, 1, data, 3, HAL_MAX_DELAY);

    if (code != HAL_OK)
    {
    	printf("[I2CWrapper] - Error %d while trying to read %d bytes from REG %d\r\n", code, 3, reg);
    }

    return (data[0] | (data[1] << 8) | (data[2] << 16));
}

uint32_t I2CWrapper::read24Be(uint8_t reg) {
    uint8_t data[3];

    HAL_StatusTypeDef code = HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_add<<1), reg, 1, data, 3, HAL_MAX_DELAY);

    if (code != HAL_OK)
    {
    	printf("[I2CWrapper] - Error %d while trying to read %d bytes from REG %d\r\n", code, 3, reg);
    }

    return (data[2] | (data[1] << 8) | (data[0] << 16));
}

void I2CWrapper::read(uint8_t reg, uint8_t* buf, uint16_t len) {

	HAL_StatusTypeDef code = HAL_I2C_Mem_Read(hi2c, (uint16_t)(dev_add<<1), reg, 1, buf, len, HAL_MAX_DELAY);

    if (code != HAL_OK)
    {
    	printf("[I2CWrapper] - Error %d while trying to read %d bytes from REG %d\r\n", code, len, reg);
    }
}

bool I2CWrapper::isConnected()
{
	HAL_StatusTypeDef code = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(dev_add<<1), 3, 5);

	if (code != HAL_OK)
	{
		return false;
	}else{
		return true;
	}
}
