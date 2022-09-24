/*
 * Telem_out.cpp
 *
 *  Created on: Sep 23, 2022
 *      Author: heapr0ll
 */

#include "Telem_out.hpp"

TelemOut::TelemOut(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm)
	: Task(delay_s,
		peri_s,
		mstr_tick_s,
		pri,
		nm)
{
	enabled = true;
}
TelemOut::~TelemOut()
{

}

// Callback function to execute by master scheduler
bool TelemOut::taskFunction()
{
	// Transmit all data out on usb port.
	if (enabled)
	{
		// SOP
		std::string msg = "$";

		// Publish Raw IMU data.
		if (!pntr_imu_msg->locked)
		{
			msg += std::to_string(pntr_imu_msg->gx) + "," +
					std::to_string(pntr_imu_msg->gy) + "," +
					std::to_string(pntr_imu_msg->gz) + "," +
					std::to_string(pntr_imu_msg->ax) + "," +
					std::to_string(pntr_imu_msg->ay) + "," +
					std::to_string(pntr_imu_msg->az) + "," +
					std::to_string(pntr_imu_msg->temp) + ",";
		}

		// Publish Batt data.
		if (!pntr_batt_msg->locked)
		{
			msg += std::to_string(pntr_batt_msg->cell1) + "," +
					std::to_string(pntr_batt_msg->cell2) + "," +
					std::to_string(pntr_batt_msg->cell3) + "," +
					std::to_string(pntr_batt_msg->cell4) + "," +
					std::to_string(pntr_batt_msg->pack_volt) + "*";
		}

		// EOP
		msg += "\n";

		// Transmit the data.
		printf(msg.c_str());
	}
}

// Callback functions to execute on console request.
bool TelemOut::consoleFunca()
{
	// Print usage instructions!
	println("Description:");
	println("Start or stop the telemetry output process running @ 100Hz.");
	println("\tArgs");
	println("\t* on - Turn telemetry out on.");
	println("\t* off - Turn telemetry out off.");
	return true;
}
bool TelemOut::consoleFuncb(std::string& s)
{
	// Check argument string.
	if (s == "on")
	{
		enabled = true;

	}else if (s == "off") {
		enabled = false;
	}else{
		println("Invalid parameter!");
		consoleFunca();
		return false;
	}

	return true;
}
