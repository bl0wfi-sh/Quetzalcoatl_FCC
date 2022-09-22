/*
 * CompFilter.cpp
 *
 *  Created on: Sep 6, 2022
 *      Author: heapr0ll
 */

#include "CompFilter.hpp"

CompFilter::CompFilter(int size) : y(1)
{
	for(int i = 0; i < size; i++)
	{
		x.push_back(i);
	}
}

CompFilter::~CompFilter()
{
	x.clear();
}

int CompFilter::operator+(const CompFilter& tmp)
{
	return this->y + tmp.y;
}

