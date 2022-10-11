/*
 * BaroMagDriver.hpp
 *
 *  Created on: Sep 30, 2022
 *      Author: heapr0ll
 */

#ifndef INC_BAROMAGDRIVER_HPP_
#define INC_BAROMAGDRIVER_HPP_

#include <stdio.h>
#include "main.h"
#include "Task.hpp"
#include "I2CWrapper.hpp"

#define DPS310_DEV_ADDRESS	0x76

// CONFIG REGISTERS
#define DPS310_PSR_CFG		0x06
#define DPS310_TMP_CFG		0x07
#define DPS310_MEAS_CFG		0x08
#define DPS310_FIFO_CFG		0x09
#define DPS310_COEF_SRC		0x28

// INTERRUPT REGISTERS
#define DPS310_INT_STS		0x0A

// FIFO REGISTERS
#define DPS310_FIFO_STS		0x0B

// RESET REGISTERS
#define DPS310_RESET		0x0C

// DATA REGISTERS (8 bit registers)
#define DPS310_PSR_B2		0x00		// Highest byte
#define DPS310_PSR_B1		0x01
#define DPS310_PSR_B0		0x02		// Lowest byte
#define DPS310_TMP_B2		0x03		// Highest byte
#define DPS310_TMP_B1		0x04
#define DPS310_TMP_B0		0x05		// Lowest byte

// SENSOR CONFIGURATION
#define DPS310_SNS_MRATE_OFF	0x04
enum sens_mrate_type_t {
    MPS_1 = 0x00, //
	MPS_2 = 0x01,
	MPS_4 = 0x02,
	MPS_8 = 0x03,
	MPS_16 = 0x04,
	MPS_32 = 0x05,
	MPS_64 = 0x06,
	MPS_128 = 0x07
};

#define DPS310_SNS_OSRATE_OFF	0x00
enum sens_osrate_type_t {
    SPS_1 = 0x00, //
	SPS_2 = 0x01,
	SPS_4 = 0x02,
	SPS_8 = 0x03,
	SPS_16 = 0x04,
	SPS_32 = 0x05,
	SPS_64 = 0x06,
	SPS_128 = 0x07
};

// TEMP SENSOR CONFIGURATION
#define DPS310_TMP_M_OFF	0x07
enum tmp_msrc_typ_t {
	MEAS_SRC_INT = 0x00,
	MEAS_SRC_EXT = 0x01
};

// MODE CONFIGURATION
enum mode_type_t {
	STANDBY_MODE = 0x00,
	COMMAND_PRS_MODE = 0x01,
	COMMAND_TMP_MODE = 0x02,
	CONT_PRS_MODE = 0x05,
	CONT_TMP_MODE = 0x06,
	CONT_PRS_TMP_MODE = 0x07
};

// FIFO/INTERRUPT CONFIGURATION
enum fifo_cnfg_type_t {
	SPI_MODE = 0x00,
	FIFO_EN = 0x01,
	P_SHIFT = 0x02,
	T_SHIFT = 0x03,
	INT_PRS = 0x04,
	INT_TMP = 0x05,
	INT_FIFO = 0x06,
	INT_HL = 0x07
};

// Driver class for DPS310
class DPS310
{
public:
	DPS310();
	~DPS310();

	bool init(I2C_HandleTypeDef* i2c);

	// Utilit functions to set things up
	void setMRatePRS(uint8_t rate);
	void setOSRatePRS(uint8_t rate);

	void setMRateTMP(uint8_t rate);
	void setOSRateTMP(uint8_t rate);
	void setTMPSource(uint8_t src);

	void setOperationMode(uint8_t mode);

	void setFifoCnfgBit(uint8_t bitoffset, uint8_t value);

	void setCoefSrc(uint8_t src);

private:
	I2CWrapper my_i2c;
};

// Driver class for MMC5603
class MMC5603
{
public:
private:
};

// Task class for Baro + Mag sensing.
class BaroMag : public Task
{
public:
	BaroMag(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm);
	~BaroMag();

	bool init(I2C_HandleTypeDef* i2c)
	{
		return mybaro.init(i2c);
	}

	// virtual bool taskFunction();

	//virtual bool consoleFunca();
	//virtual bool consoleFuncb(std::string& s);

private:
	DPS310 mybaro;
};

#endif /* INC_BAROMAGDRIVER_HPP_ */
