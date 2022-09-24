/*
 * BatteryMon.hpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#ifndef INC_BATTERYMON_HPP_
#define INC_BATTERYMON_HPP_

#include "main.h"
#include <stdio.h>
#include "Task.hpp"
#include "uTopics.hpp"

// All of these resistor values are in killo ohms.
#define CELL1_R1	100.0f
#define CELL1_R2	100.0f
#define CELL2_R1	300.0f
#define CELL2_R2	100.0f
#define CELL3_R1	500.0f
#define CELL3_R2	100.0f
#define CELL4_R1	698.0f
#define CELL4_R2	100.0f

class BattMon : public Task
{
public:
	BattMon(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm);
	~BattMon();
	void init(ADC_HandleTypeDef* hadc);

	virtual bool taskFunction();

	virtual bool consoleFunca();
	virtual bool consoleFuncb(std::string& s);

	// Utility functions to setup analog channels.
	void setupCell1();
	void setupCell2();
	void setupCell3();
	void setupCell4();

	// Utility functions to read cells.
	float readCell1();
	float readCell2();
	float readCell3();
	float readCell4();

private:

	ADC_HandleTypeDef* hadc;
	batt_msg_struct* batt_msg_pntr = &sys_batt_topic;
};


#endif /* INC_BATTERYMON_HPP_ */
