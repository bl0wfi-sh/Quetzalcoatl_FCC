/*
 * Task.cpp
 *
 *  Created on: Sep 16, 2022
 *      Author: heapr0ll
 */

#include "Task.hpp"

Task::Task()
	: state(-1), priority(0), delay_tk(0), period_tk(0), name("none"), runtime_tk(0)
{

}

Task::Task(	int st, char pri, unsigned long del, unsigned long per)
	: state{st}, priority{pri}, delay_tk{del}, period_tk{per}
{

}

Task::Task(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm)
{
	state = 0;
	priority = pri;
	name = nm;

	// Calculating delay and period based on master tick value.
	delay_tk = (unsigned long)(delay_s / mstr_tick_s);
	period_tk = (unsigned long)(peri_s / mstr_tick_s);
}

Task::Task(Task&& other) noexcept
	: 	state(other.state), priority(other.priority), delay_tk(other.delay_tk),
		period_tk(other.period_tk)
{

}

Task& Task::operator=(Task&& other) noexcept
{
	// If objects are the same do nothing.
	if (this != &other)
	{
		state = other.state;
		priority = other.priority;
		delay_tk = other.delay_tk;
		period_tk = other.period_tk;
	}

	return *this;
}



