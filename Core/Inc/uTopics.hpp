/*
 * uTopics.hpp
 *
 *  Created on: Sep 23, 2022
 *      Author: heapr0ll
 */

#ifndef INC_UTOPICS_HPP_
#define INC_UTOPICS_HPP_

#include "main.h"

// Define IMU topic
typedef struct imu_msg imu_msg_struct;
struct imu_msg
{

	union
	{
		struct
		{
			float gx;
			float gy;
			float gz;
		};

		struct
		{
			float gyros[3];
		};
	};

	union
	{
		struct
		{
			float ax;
			float ay;
			float az;
		};

		struct
		{
			float accels[3];
		};
	};

	uint16_t temp;

	bool locked = false;
};
extern imu_msg_struct sys_imu_topic;

// Define Battery Mon topic
typedef struct batt_msg batt_msg_struct;
struct batt_msg
{
	union
	{
		struct
		{
			float cell1;
			float cell2;
			float cell3;
			float cell4;
		};

		struct
		{
			float cvolts[4];
		};
	};

	float pack_volt;

	bool locked = false;
};
extern batt_msg_struct sys_batt_topic;

#endif /* INC_UTOPICS_HPP_ */
