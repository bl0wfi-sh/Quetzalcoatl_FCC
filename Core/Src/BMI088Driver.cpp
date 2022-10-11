/*
 * BMI088Driver.cpp
 *
 *  Created on: Sep 16, 2022
 *      Author: heapr0ll
 */
#include "BMI088Driver.hpp"

BMI088::BMI088(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm)
	: Task(delay_s,
			peri_s,
			mstr_tick_s,
			pri,
			nm)
{
	devAddrAcc = BMI088_ACC_ADDRESS;
	devAddrGyro = BMI088_GYRO_ADDRESS;
	gcal_x = 0;
	gcal_y = 0;
	gcal_z = 0;

	// Initialize low pass filters.
	// Default = 35Hz for gyros, 45Hz accels.
	lpfs[0].setCutOff(35.0f, peri_s);
	lpfs[1].setCutOff(35.0f, peri_s);
	lpfs[2].setCutOff(35.0f, peri_s);

	lpfs[3].setCutOff(45.0f, peri_s);
	lpfs[4].setCutOff(45.0f, peri_s);
	lpfs[5].setCutOff(45.0f, peri_s);
}

bool BMI088::consoleFunca()
{
	// Print usage instructions!
	println("Description:");
	println("Read BMI088 accel, gyro, and temp data @ 800Hz.");
	println("\tArgs");
	println("\t* status - Show most recent accel, gyro, and temp data.");
	println("\t* filter_on - Turn data filter on.");
	println("\t* filter_off - Turn data filter off.");
	return true;
}

bool BMI088::consoleFuncb(std::string& s)
{
	// Check argument string.
	if (s == "status")
	{

		if (!imu_msg_pntr->locked)
		{
			println("BMI088 Filter Status - " + std::to_string(filter_on));
			println("BMI088 Filter (Hz) [Gyro, Accel]");
			println("BMI088 Filters - " +
					std::to_string(lpfs[0].getCutoff()) + ", " +
					std::to_string(lpfs[3].getCutoff())
					);

			println("BMI088 Units - [rads, m/s^2, C]");
			println("BMI088 Frmt  - [gx, gy, gz, ax, ay, az, temp]");
			println("BMI088 Data  - " +
					std::to_string(imu_msg_pntr->gx) + " " +
					std::to_string(imu_msg_pntr->gy) + " " +
					std::to_string(imu_msg_pntr->gz) + " " +
					std::to_string(imu_msg_pntr->ax) + " " +
					std::to_string(imu_msg_pntr->ay) + " " +
					std::to_string(imu_msg_pntr->az) + " " +
					std::to_string(imu_msg_pntr->temp)
					);
		}else{
			println("Shared memory locked!");
		}

	}else if (s == "filter_on"){

		filter_on = true;

	}else if (s == "filter_off"){

		filter_on = false;

	}else{
		println("Invalid parameter!");
		consoleFunca();
		return false;
	}

	return true;
}

bool BMI088::taskFunction()
{
	float gy[3] = {0};
	float ax[3] = {0};

	getGyroscope(gy, gy + 1, gy + 2);
	getAcceleration(ax, ax + 1, ax + 2);

	imu_msg_pntr->locked = true;			// Lock the shared memory while writting to it.

	if (filter_on)
	{
		// Low pass filter gyro data.
		imu_msg_pntr->gx = lpfs[0].filter(gy[0]);
		imu_msg_pntr->gy = lpfs[1].filter(gy[1]);
		imu_msg_pntr->gz = lpfs[2].filter(gy[2]);

		// Low pass filter accel data.
		imu_msg_pntr->ax = lpfs[3].filter(ax[0]);
		imu_msg_pntr->ay = lpfs[4].filter(ax[1]);
		imu_msg_pntr->az = lpfs[5].filter(ax[2]);
	}else{
		imu_msg_pntr->gx = gy[0];
		imu_msg_pntr->gy = gy[1];
		imu_msg_pntr->gz = gy[2];

		// Low pass filter accel data.
		imu_msg_pntr->ax = ax[0];
		imu_msg_pntr->ay = ax[1];
		imu_msg_pntr->az = ax[2];
	}

	imu_msg_pntr->temp = getTemperature();
	imu_msg_pntr->locked = false;			// Un-lock the shared memory while writting to it.
	return true;
}

bool BMI088::initialize(FMPI2C_HandleTypeDef* i) {

	i2c = i;

	// Check if is connected.
	if (!isConnection()) return false;

    setAccScaleRange(RANGE_3G);
    setAccOutputDataRate(ODR_1600);
    setAccLowPassFilterBandwidth(BWP_4_FOLD);   // Refer to data sheet to see how Low Pass Filter is set based on Output Data Rate and this register value.
    setAccPoweMode(ACC_ACTIVE);

    setGyroScaleRange(RANGE_500);
    setGyroOutputDataRate(ODR_2000_BW_230);
    setGyroPoweMode(GYRO_NORMAL);

    calibrateGyro(5000);

    return true;
}

void BMI088::calibrateGyro(int count){
    float x_tot = 0, y_tot = 0, z_tot = 0;
    for(int i = 0; i < count; i++){
      float gx, gy, gz;
      getGyroscope(&gx, &gy, &gz);
      x_tot += gx;
      y_tot += gy;
      z_tot += gz;
    }
    gcal_x = x_tot / count;
    gcal_y = y_tot / count;
    gcal_z = z_tot / count;
}

bool BMI088::isConnection(void) {
    return ((getAccID() == 0x1E) && (getGyroID() == 0x0F));
}

void BMI088::resetAcc(void) {
    write8(ACC, BMI088_ACC_SOFT_RESET, 0xB6);
}

void BMI088::resetGyro(void) {
    write8(GYRO, BMI088_GYRO_SOFT_RESET, 0xB6);
}

uint8_t BMI088::getAccID(void) {
    return read8(ACC, BMI088_ACC_CHIP_ID);
}

uint8_t BMI088::getGyroID(void) {
    return read8(GYRO, BMI088_GYRO_CHIP_ID);
}

void BMI088::setAccPoweMode(acc_power_type_t mode) {
    if (mode == ACC_ACTIVE) {
        write8(ACC, BMI088_ACC_PWR_CTRl, 0x04);
        write8(ACC, BMI088_ACC_PWR_CONF, 0x00);
    } else if (mode == ACC_SUSPEND) {
        write8(ACC, BMI088_ACC_PWR_CONF, 0x03);
        write8(ACC, BMI088_ACC_PWR_CTRl, 0x00);
    }
}

void BMI088::setGyroPoweMode(gyro_power_type_t mode) {
    if (mode == GYRO_NORMAL) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
    } else if (mode == GYRO_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
    } else if (mode == GYRO_DEEP_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
    }
}

void BMI088::setAccScaleRange(acc_scale_type_t range) {
    if (range == RANGE_3G) {
        accRange = 3;
    } else if (range == RANGE_6G) {
        accRange = 6;
    } else if (range == RANGE_12G) {
        accRange = 12;
    } else if (range == RANGE_24G) {
        accRange = 24;
    }

    write8(ACC, BMI088_ACC_RANGE, (uint8_t)range);
}

void BMI088::setAccOutputDataRate(acc_odr_type_t odr) {
    uint8_t data = 0;

    data = read8(ACC, BMI088_ACC_CONF);
    data = data & 0xf0;
    data = data | (uint8_t)odr;

    write8(ACC, BMI088_ACC_CONF, data);
}

void BMI088::setAccLowPassFilterBandwidth(acc_bwp_type_t bwp) {
    uint8_t data = 0;

    data = read8(ACC, BMI088_ACC_CONF);
    data = data & 0x0f;                   // Wipe only top 4 bits
    data = data | (uint8_t)(bwp << 4);    // Low pass filter is bits [7:4]

    write8(ACC, BMI088_ACC_CONF, data);
}


void BMI088::setGyroScaleRange(gyro_scale_type_t range) {
    if (range == RANGE_2000) {
        gyroRange = 2000;
    } else if (range == RANGE_1000) {
        gyroRange = 1000;
    } else if (range == RANGE_500) {
        gyroRange = 500;
    } else if (range == RANGE_250) {
        gyroRange = 250;
    } else if (range == RANGE_125) {
        gyroRange = 125;
    }

    write8(GYRO, BMI088_GYRO_RANGE, (uint8_t)range);
}

void BMI088::setGyroOutputDataRate(gyro_odr_type_t odr) {
    write8(GYRO, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}

void BMI088::getAcceleration(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t ax = 0, ay = 0, az = 0;
    float value = 0;

    read(ACC, BMI088_ACC_X_LSB, buf, 6);

    ax = buf[0] | (buf[1] << 8);
    ay = buf[2] | (buf[3] << 8);
    az = buf[4] | (buf[5] << 8);

    value = (int16_t)ax;
    *x = accRange * value / 32768;

    value = (int16_t)ay;
    *y = accRange * value / 32768;

    value = (int16_t)az;
    *z = accRange * value / 32768;
}

float BMI088::getAccelerationX(void) {
    uint16_t ax = 0;
    float value = 0;

    ax = read16(ACC, BMI088_ACC_X_LSB);

    value = (int16_t)ax;
    value = accRange * value / 32768;

    return value;
}

float BMI088::getAccelerationY(void) {
    uint16_t ay = 0;
    float value = 0;

    ay = read16(ACC, BMI088_ACC_Y_LSB);

    value = (int16_t)ay;
    value = accRange * value / 32768;

    return value;
}

float BMI088::getAccelerationZ(void) {
    uint16_t az = 0;
    float value = 0;

    az = read16(ACC, BMI088_ACC_Z_LSB);

    value = (int16_t)az;
    value = accRange * value / 32768;

    return value;
}

void BMI088::getGyroscope(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t gx = 0, gy = 0, gz = 0;
    float value = 0;

    read(GYRO, BMI088_GYRO_RATE_X_LSB, buf, 6);

    gx = buf[0] | (buf[1] << 8);
    gy = buf[2] | (buf[3] << 8);
    gz = buf[4] | (buf[5] << 8);

    value = (int16_t)gx;
    *x = (gyroRange * value / 32768) - gcal_x;

    value = (int16_t)gy;
    *y = -(gyroRange * value / 32768) - gcal_y;

    value = (int16_t)gz;
    *z = -(gyroRange * value / 32768) - gcal_z;
}

float BMI088::getGyroscopeX(void) {
    uint16_t gx = 0;
    float value = 0;

    gx = read16(GYRO, BMI088_GYRO_RATE_X_LSB);

    value = (int16_t)gx;
    value = (gyroRange * value / 32768) - gcal_x;

    return value;
}

float BMI088::getGyroscopeY(void) {
    uint16_t gy = 0;
    float value = 0;

    gy = read16(GYRO, BMI088_GYRO_RATE_Y_LSB);

    value = (int16_t)gy;
    value = (gyroRange * value / 32768) - gcal_y;

    return value * -1.0f;
}

float BMI088::getGyroscopeZ(void) {
    uint16_t gz = 0;
    float value = 0;

    gz = read16(GYRO, BMI088_GYRO_RATE_Z_LSB);

    value = (int16_t)gz;
    value = (gyroRange * value / 32768) - gcal_z;

    return value * -1.0f;
}

int16_t BMI088::getTemperature(void) {
    uint16_t data = 0;

    data = read16Be(ACC, BMI088_ACC_TEMP_MSB);
    data = data >> 5;

    if (data > 1023) {
        data = data - 2048;
    }

    return (int16_t)(data / 8 + 23);
}

void BMI088::write8(device_type_t dev, uint8_t reg, uint8_t val) {
    uint8_t addr = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    HAL_FMPI2C_Mem_Write(i2c, (uint16_t)(addr<<1), reg, 1, &val, 1, HAL_MAX_DELAY);
}

uint8_t BMI088::read8(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0, data = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    HAL_StatusTypeDef code = HAL_FMPI2C_Mem_Read(i2c, (uint16_t)(addr<<1), reg, 1, &data, 1, HAL_MAX_DELAY);
    if (code != HAL_OK)
    {
    	printf("[BMI088] - Error %d while trying to read %d bytes from REG %d\r\n", code, 1, reg);
    }

    return data;
}

uint16_t BMI088::read16(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint8_t data[2];

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    HAL_FMPI2C_Mem_Read(i2c, (uint16_t)(addr<<1), reg, 1, data, 2, HAL_MAX_DELAY);

    return (data[0] | (data[1] << 8));
}

uint16_t BMI088::read16Be(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint8_t data[2];

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    HAL_FMPI2C_Mem_Read(i2c, (uint16_t)(addr<<1), reg, 1, data, 2, HAL_MAX_DELAY);

    return (data[1] | (data[0] << 8));
}

uint32_t BMI088::read24(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint8_t data[3];

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    HAL_FMPI2C_Mem_Read(i2c, (uint16_t)(addr<<1), reg, 1, data, 3, HAL_MAX_DELAY);

    return (data[0] | (data[1] << 8) | (data[2] << 16));
}

void BMI088::read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len) {
    uint8_t addr = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    HAL_FMPI2C_Mem_Read(i2c, (uint16_t)(addr<<1), reg, 1, buf, len, HAL_MAX_DELAY);
}
