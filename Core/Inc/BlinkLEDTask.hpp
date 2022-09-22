/*
 * BlinkLEDTask.hpp
 *
 *  Created on: Sep 16, 2022
 *      Author: heapr0ll
 */

#ifndef INC_BLINKLEDTASK_HPP_
#define INC_BLINKLEDTASK_HPP_

#include "main.h"
#include <stdio.h>
#include "Task.hpp"

class BlinkTask : public Task
{
public:
	BlinkTask(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm);
	~BlinkTask();

	virtual bool taskFunction();

	//virtual bool consoleFunca();
	//virtual bool consoleFuncb(std::string& s);

private:
};



#endif /* INC_BLINKLEDTASK_HPP_ */
