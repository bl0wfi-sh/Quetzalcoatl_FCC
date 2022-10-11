/*
 * BaroMagDriver.cpp
 *
 *  Created on: Sep 30, 2022
 *      Author: heapr0ll
 */

#include "BaroMagDriver.hpp"

DPS310::DPS310()
	: my_i2c()
{

}

DPS310::~DPS310()
{

}

bool DPS310::init(I2C_HandleTypeDef* i2c)
{
	my_i2c.init(i2c);
	my_i2c.setDevAdd(DPS310_DEV_ADDRESS);

	// Check if device is connected.
	if ( !my_i2c.isConnected() )
	{
		return false;
	}

	/**

	// Set Coefficient Configuration
	setCoefSrc(MEAS_SRC_EXT);

	// Set FIFO Configuration
	setFifoCnfgBit(SPI_MODE, 0);
	setFifoCnfgBit(FIFO_EN,  1);
	setFifoCnfgBit(P_SHIFT,  1);
	setFifoCnfgBit(T_SHIFT,  0);
	setFifoCnfgBit(INT_PRS,  0);
	setFifoCnfgBit(INT_TMP,  0);
	setFifoCnfgBit(INT_FIFO, 0);
	setFifoCnfgBit(INT_HL,   0);

	// Set operational mode.
	setOperationMode(CONT_PRS_MODE);

	// Set Pressure Sensor Configuration
	setMRatePRS(MPS_4);
	setOSRatePRS(SPS_64);

	// Set Temperature Sensor Configuration
	setMRateTMP(MPS_4);
	setOSRateTMP(SPS_1);
	setTMPSource(MEAS_SRC_EXT);

	**/

	return true;
}

/**
 * Pressure sensor functions.
 */
void DPS310::setMRatePRS(uint8_t rate)
{
	uint8_t val = my_i2c.read8(DPS310_PSR_CFG);

	// Over writing top 4 bits of byte.
	// Bit 7 is a reserve so we keep that.
	val = val & 0x8f;
	val = val | rate;

	this->my_i2c.write8(DPS310_PSR_CFG, val);
}
void DPS310::setOSRatePRS(uint8_t rate)
{
	uint8_t val = my_i2c.read8(DPS310_PSR_CFG);

	// Over writing bottom 4 bits of byte.
	val = val & 0xf0;
	val = val | rate;

	this->my_i2c.write8(DPS310_PSR_CFG, val);
}

/**
 * Temperature sensor functions.
 */
void DPS310::setMRateTMP(uint8_t rate)
{
	uint8_t val = my_i2c.read8(DPS310_TMP_CFG);

	// Over writing top 4 bits of byte.
	// Bit 7 is for setting measurement source.
	val = val & 0x8f;
	val = val | rate;

	this->my_i2c.write8(DPS310_TMP_CFG, val);
}
void DPS310::setOSRateTMP(uint8_t rate)
{
	uint8_t val = my_i2c.read8(DPS310_TMP_CFG);

	// Over writing bottom 4 bits of byte.
	val = val & 0xf0;
	val = val | rate;

	this->my_i2c.write8(DPS310_TMP_CFG, val);
}
void DPS310::setTMPSource(uint8_t src)
{
	uint8_t val = my_i2c.read8(DPS310_TMP_CFG);

	// Over writing bottom 4 bits of byte.
	val = val & 0x7f;
	val = val | (src << DPS310_TMP_M_OFF);

	this->my_i2c.write8(DPS310_TMP_CFG, val);
}

/**
 * Sensor Operating mode functions.
 */
void DPS310::setOperationMode(uint8_t mode)
{
	uint8_t val = my_i2c.read8(DPS310_MEAS_CFG);

	// Over writing bottom 4 bits of byte.
	// Keep MSB of bottom 4 bits as its reserved.
	val = val & 0xf8;
	val = val | mode;

	this->my_i2c.write8(DPS310_MEAS_CFG, val);
}

/**
 * Fifo Configuration functions.
 */
void DPS310::setFifoCnfgBit(uint8_t bitoffset, uint8_t value)
{
	uint8_t val = my_i2c.read8(DPS310_FIFO_CFG);

	// Only allow writing a 1, or 0
	if (value == 0)
	{
		val = val & ~(value << bitoffset);
		this->my_i2c.write8(DPS310_FIFO_CFG, val);

	}else if (value == 1)
	{
		val = val | (value << bitoffset);
		this->my_i2c.write8(DPS310_FIFO_CFG, val);
	}
}

/**
 * Coefficient Configuration functions.
 */
void DPS310::setCoefSrc(uint8_t src)
{
	uint8_t val = my_i2c.read8(DPS310_COEF_SRC);

	// Only allow writing a 1, or 0
	if (src == 0)
	{
		val = val & ~(src << DPS310_TMP_M_OFF);
		this->my_i2c.write8(DPS310_COEF_SRC, val);

	}else if (src == 1)
	{
		val = val | (src << DPS310_TMP_M_OFF);
		this->my_i2c.write8(DPS310_COEF_SRC, val);
	}
}


BaroMag::BaroMag(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm)
	: Task(delay_s,
			peri_s,
			mstr_tick_s,
			pri,
			nm),
	  mybaro()
{

}

BaroMag::~BaroMag()
{

}
