/*
 * BatteryMon.cpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#include "BatteryMon.hpp"

BattMon::BattMon(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm)
	: Task(delay_s,
			peri_s,
			mstr_tick_s,
			pri,
			nm)
{

}
BattMon::~BattMon()
{

}

void BattMon::init(ADC_HandleTypeDef* adc)
{
	hadc = adc;
}

bool BattMon::consoleFunca()
{
	// Print usage instructions!
	println("Description:");
	println("Read 4s Lipo Cell Voltages @ 4Hz.");
	println("\tArgs");
	println("\t* status - Shows most recent pack voltage and individual cell voltages.");
	return true;
}

bool BattMon::consoleFuncb(std::string& s)
{
	// Check argument string.
	if (s == "status")
	{
		println("Lipo Pack Units - [Volts]");
		println("Lipo Pack Frmt  - [c1, c2, c3, c4, total]");
		println("Lipo Pack Data  - " +
				std::to_string(cell_voltages.cell1) + " " +
				std::to_string(cell_voltages.cell2) + " " +
				std::to_string(cell_voltages.cell3) + " " +
				std::to_string(cell_voltages.cell4) + " " +
				std::to_string(pack_volt));
	}else{
		println("Invalid parameter!");
		consoleFunca();
		return false;
	}

	return true;
}

bool BattMon::taskFunction()
{
	// Read all cells.
	readCell1();
	readCell2();
	readCell3();
	readCell4();
	pack_volt = cell_voltages.cell4 + cell_voltages.cell3 + cell_voltages.cell2 + cell_voltages.cell1;
	return true;
}

void BattMon::setupCell1()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void BattMon::setupCell2()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void BattMon::setupCell3()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void BattMon::setupCell4()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void BattMon::readCell1()
{
	setupCell1();
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	cell_voltages.cell1 = ((float)HAL_ADC_GetValue(hadc) * 3.3f * (CELL1_R1 + CELL1_R2)) / (CELL1_R2 * 4096.0f);
	HAL_ADC_Stop(hadc);
}

void BattMon::readCell2()
{
	setupCell2();
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	cell_voltages.cell2 = ((float)HAL_ADC_GetValue(hadc) * 3.3f * (CELL2_R1 + CELL2_R2)) / (CELL2_R2 * 4096.0f);
	cell_voltages.cell2 -= cell_voltages.cell1;
	HAL_ADC_Stop(hadc);
}

void BattMon::readCell3()
{
	setupCell3();
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	cell_voltages.cell3 = ((float)HAL_ADC_GetValue(hadc) * 3.3f * (CELL3_R1 + CELL3_R2)) / (CELL3_R2 * 4096.0f);
	cell_voltages.cell3 -= cell_voltages.cell2 + cell_voltages.cell1;
	HAL_ADC_Stop(hadc);
}

void BattMon::readCell4()
{
	setupCell4();
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1000);
	cell_voltages.cell4 = ((float)HAL_ADC_GetValue(hadc) * 3.3f * (CELL4_R1 + CELL4_R2)) / (CELL4_R2 * 4096.0f);
	cell_voltages.cell4 -= cell_voltages.cell3 + cell_voltages.cell2 + cell_voltages.cell1;
	HAL_ADC_Stop(hadc);
}
