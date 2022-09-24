/*
 * Telem_out.hpp
 *
 *  Created on: Sep 23, 2022
 *      Author: heapr0ll
 */

#ifndef INC_TELEM_OUT_HPP_
#define INC_TELEM_OUT_HPP_

#include <string>
#include "Task.hpp"
#include "uTopics.hpp"

extern imu_msg_struct sys_imu_topic;
extern batt_msg_struct sys_batt_topic;

class TelemOut : public Task
{
public:
	TelemOut(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm);
	~TelemOut();

	// Callback function to execute by master scheduler
	virtual bool taskFunction();

	// Callback functions to execute on console request.
	virtual bool consoleFunca();
	virtual bool consoleFuncb(std::string& s);

private:
	bool enabled;
	batt_msg_struct* pntr_batt_msg = &sys_batt_topic;
	imu_msg_struct* pntr_imu_msg = &sys_imu_topic;
};


#endif /* INC_TELEM_OUT_HPP_ */
