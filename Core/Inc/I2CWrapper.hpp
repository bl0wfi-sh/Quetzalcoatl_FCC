/*
 * I2CWrapper.hpp
 *
 *  Created on: Sep 30, 2022
 *      Author: heapr0ll
 */

#ifndef INC_I2CWRAPPER_HPP_
#define INC_I2CWRAPPER_HPP_

#include "main.h"
#include <stdio.h>

// I2C class.
class I2CWrapper
{
public:
	I2CWrapper()
		: dev_add(0)
	{

	}

	I2CWrapper(uint16_t addy)
		:dev_add(addy)
	{

	}

	~I2CWrapper();

	void init(I2C_HandleTypeDef* i2c)
	{
		hi2c = i2c;
	}

	// Setters
	void setDevAdd(uint16_t add) { dev_add = add; }

	// Getters
	uint16_t getDevAdd() { return dev_add; }

	// Utility functions for reading I2C bus.
	void write8(uint8_t reg, uint8_t val);
	uint8_t read8(uint8_t reg);
	uint16_t read16(uint8_t reg);
	uint16_t read16Be(uint8_t reg);
	uint32_t read24(uint8_t reg);
	uint32_t read24Be(uint8_t reg);
	void read(uint8_t reg, uint8_t* buf, uint16_t len);
	bool isConnected();

private:
	uint16_t dev_add;
	I2C_HandleTypeDef* hi2c;
};


#endif /* INC_I2CWRAPPER_HPP_ */
