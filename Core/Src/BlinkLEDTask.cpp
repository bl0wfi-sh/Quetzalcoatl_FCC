/*
 * BlinkLEDTask.cpp
 *
 *  Created on: Sep 16, 2022
 *      Author: heapr0ll
 */

#include "BlinkLEDTask.hpp"

BlinkTask::BlinkTask(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm)
	: Task(delay_s,
			peri_s,
			mstr_tick_s,
			pri,
			nm)
{

}
BlinkTask::~BlinkTask()
{

}

bool BlinkTask::taskFunction()
{
	HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
	return true;
}

