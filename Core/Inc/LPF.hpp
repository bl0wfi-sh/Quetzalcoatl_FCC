/*
 * LPF.hpp
 *
 *  Created on: Sep 30, 2022
 *      Author: heapr0ll
 */

#ifndef INC_LPF_HPP_
#define INC_LPF_HPP_

#include <math.h>

class LPF
{
public:
	LPF()
		: cutoff(0), alpha(0), last_out(0)
	{

	}

	LPF(float cut, float dt)
		:cutoff(cut), last_out(0)
	{
		this->alpha = exp(-cut*dt);
	}

	~LPF();

	float filter(float meas)
	{
		last_out = alpha*last_out + (1 - alpha)*meas;
		return last_out;
	}

	// Setters if needed.
	void setCutOff(float cut, float dt)			// Setting a new cutoff frequency will reset the filter to 0.
	{
		cutoff = cut;
		alpha = exp(-cut*dt);
		last_out = 0;
	}

	// Getter if needed.
	float getCutoff(){ return cutoff; }
	float getAlpha(){ return alpha; }
	float getLast(){ return last_out; }

private:
	float cutoff;
	float alpha;
	float last_out;
};


#endif /* INC_LPF_HPP_ */
