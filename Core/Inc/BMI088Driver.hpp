/*
 * BMI088Driver.hpp
 *
 *  Created on: Sep 16, 2022
 *      Author: heapr0ll
 */

#ifndef INC_BMI088DRIVER_HPP_
#define INC_BMI088DRIVER_HPP_

#include <stdio.h>
#include "Task.hpp"
#include "main.h"
#include "uTopics.hpp"
#include "LPF.hpp"

#define BMI088_ACC_ADDRESS          0x18

#define BMI088_ACC_CHIP_ID          0x00 // Default value 0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_X_MSB            0x13
#define BMI088_ACC_Y_LSB            0x14
#define BMI088_ACC_Y_MSB            0x15
#define BMI088_ACC_Z_LSB            0x16
#define BMI088_ACC_Z_MSB            0x17

#define BMI088_ACC_SENSOR_TIME_0    0x18
#define BMI088_ACC_SENSOR_TIME_1    0x19
#define BMI088_ACC_SENSOR_TIME_2    0x1A

#define BMI088_ACC_INT_STAT_1       0x1D

#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41

#define BMI088_ACC_INT1_IO_CTRL     0x53
#define BMI088_ACC_INT2_IO_CTRL     0x54
#define BMI088_ACC_INT_MAP_DATA     0x58

#define BMI088_ACC_SELF_TEST        0x6D

#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRl         0x7D

#define BMI088_ACC_SOFT_RESET       0x7E

#define BMI088_GYRO_ADDRESS             0x68

#define BMI088_GYRO_CHIP_ID             0x00 // Default value 0x0F

#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07

#define BMI088_GYRO_INT_STAT_1          0x0A

#define BMI088_GYRO_RANGE               0x0F
#define BMI088_GYRO_BAND_WIDTH          0x10

#define BMI088_GYRO_LPM_1               0x11

#define BMI088_GYRO_SOFT_RESET          0x14

#define BMI088_GYRO_INT_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18

#define BMI088_GYRO_SELF_TEST           0x3C

enum device_type_t { // device type
    ACC = 0x00, //
    GYRO = 0x01, //
};

enum acc_scale_type_t { // measurement rage
    RANGE_3G = 0x00, //
    RANGE_6G = 0x01, //
    RANGE_12G = 0x02, //
    RANGE_24G = 0x03, //
};

enum acc_bwp_type_t { // Bandwidth of low pass filter
    BWP_4_FOLD = 0x08,
    BWP_2_FOLD = 0x09,
    BWP_NORM = 0x0A,
};

enum acc_odr_type_t { // output data rate
    ODR_12 = 0x05, //
    ODR_25 = 0x06, //
    ODR_50 = 0x07, //
    ODR_100 = 0x08, //
    ODR_200 = 0x09, //
    ODR_400 = 0x0A, //
    ODR_800 = 0x0B, //
    ODR_1600 = 0x0C, //
};

enum acc_power_type_t { // power mode
    ACC_ACTIVE = 0x00, //
    ACC_SUSPEND = 0x03, //
};

enum gyro_scale_type_t { // measurement rage
    RANGE_2000 = 0x00, //
    RANGE_1000 = 0x01, //
    RANGE_500 = 0x02, //
    RANGE_250 = 0x03, //
    RANGE_125 = 0x04, //
};

enum gyro_odr_type_t { // output data rate
    ODR_2000_BW_532 = 0x00, //
    ODR_2000_BW_230 = 0x01, //
    ODR_1000_BW_116 = 0x02, //
    ODR_400_BW_47 = 0x03, //
    ODR_200_BW_23 = 0x04, //
    ODR_100_BW_12 = 0x05, //
    ODR_200_BW_64 = 0x06, //
    ODR_100_BW_32 = 0x07, //
};

enum gyro_power_type_t { // power mode
    GYRO_NORMAL = 0x00, //
    GYRO_SUSPEND = 0x80, //
    GYRO_DEEP_SUSPEND = 0x20, //
};

class BMI088 : public Task
{
public:

	BMI088(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm);
	~BMI088();

	bool isConnection(void);

	bool initialize(FMPI2C_HandleTypeDef* i);

	void setAccPoweMode(acc_power_type_t mode);
	void setGyroPoweMode(gyro_power_type_t mode);

	void setAccScaleRange(acc_scale_type_t range);
	void setAccOutputDataRate(acc_odr_type_t odr);
	void setAccLowPassFilterBandwidth(acc_bwp_type_t bwp);

	void setGyroScaleRange(gyro_scale_type_t range);
	void setGyroOutputDataRate(gyro_odr_type_t odr);

	void getAcceleration(float* x, float* y, float* z);
	float getAccelerationX(void);
	float getAccelerationY(void);
	float getAccelerationZ(void);

	void getGyroscope(float* x, float* y, float* z);
	float getGyroscopeX(void);
	float getGyroscopeY(void);
	float getGyroscopeZ(void);

	int16_t getTemperature(void);

	uint8_t getAccID(void);
	uint8_t getGyroID(void);

	void resetAcc(void);
	void resetGyro(void);

	void calibrateGyro(int count);

	// Callback function to execute by master scheduler
	virtual bool taskFunction();

	// Callback functions to execute on console request.
	virtual bool consoleFunca();
	virtual bool consoleFuncb(std::string& s);

private:
	void write8(device_type_t dev, uint8_t reg, uint8_t val);
	uint8_t read8(device_type_t dev, uint8_t reg);
	uint16_t read16(device_type_t dev, uint8_t reg);
	uint16_t read16Be(device_type_t dev, uint8_t reg);
	uint32_t read24(device_type_t dev, uint8_t reg);
	void read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len);

	float accRange;
	float gyroRange;
	uint8_t devAddrAcc;
	uint8_t devAddrGyro;
	float gcal_x;
	float gcal_y;
	float gcal_z;

	FMPI2C_HandleTypeDef* i2c;

	// Low pass filters.
	LPF lpfs[6];	// gx, gy, gz, ax, ay, az

	// Data storage!
	imu_msg_struct* imu_msg_pntr = &sys_imu_topic;

	// Console flags
	bool filter_on = true;
};


#endif /* INC_BMI088DRIVER_HPP_ */
