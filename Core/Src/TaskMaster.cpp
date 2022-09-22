/*
 * TaskMaster.cpp
 *
 *  Created on: Sep 9, 2022
 *      Author: heapr0ll
 */

#include "TaskMaster.hpp"

// Task master object needs to be created so .cpp can have SysTick_Handler call our objects update function.
TaskMaster<10> scheduler;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == scheduler.getTimerInst())
	{
		// The timer our scheduler is using has elapsed!
		scheduler.update();
	}
}

