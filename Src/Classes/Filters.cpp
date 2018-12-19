/*
 * Filters.cpp
 *
 *  Created on: 03.01.2018
 *      Author: mice
 */

#include <Filters.h>

/* PT1 */
float PT1Filter::apply(float input) {
    state += k * (input - state);
    return state;
}
PT1Filter::PT1Filter(uint8_t f_cut, float dT){
    float RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
    k = dT / (RC + dT);
    state = 0;
}
PT1Filter::~PT1Filter() {
}

/* Biquad */
float BiquadFilter::apply(float input) {
    float result = b0 * input + x1;
    x1 = b1 * input - a1 * result + x2;
    x2 = b2 * input - a2 * result;
    return result;
}
BiquadFilter::BiquadFilter( biquadFilterType_e type, float dT, float filterFreq,  float cutoff) {

	const float omega = 2.0f * M_PI_FLOAT * filterFreq * dT;
	const float sn = sin_approx(omega);
	const float cs = cos_approx(omega);
	float Q;
	if (type == FILTER_NOTCH) {
		float octaves = log2f((float) filterFreq / (float) cutoff) * 2;
		Q = sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1);
	} else
		Q = BIQUAD_Q;
	const float alpha = sn / (2.0f * Q);

	float a0 = 0;
	b0 = 0; b1 = 0; b2 = 0; a0 = 0; a1 = 0; a2 = 0;

	switch (type) {
	case FILTER_LPF:
		b0 = (1 - cs) * 0.5f;
		b1 = 1 - cs;
		b2 = (1 - cs) * 0.5f;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case FILTER_NOTCH:
		b0 =  1;
		b1 = -2 * cs;
		b2 = 1;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case FILTER_BPF:
		b0 = alpha;
		b1 = 0;
		b2 = -alpha;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	}

	// precompute the coefficients
	b0 = b0 / a0;
	b1 = b1 / a0;
	b2 = b2 / a0;
	a1 = a1 / a0;
	a2 = a2 / a0;

	// zero initial samples
	x1 = x2 = 0;
	y1 = y2 = 0;
}
BiquadFilter::~BiquadFilter() {}

/* Kalman */
float KalmanFilter::apply(float input){

	//project the state ahead using acceleration
	x += (x - lastX);

	//update last state
	lastX = x;

	//prediction update
	p += q;

	//measurement update
	k = p / (p + r);
	x += k * (input - x);
	p = (1.f - k) * p;

	return x;
}
KalmanFilter::KalmanFilter(float qpar, float rpar, float ppar, float intialValue) {

	q     = qpar * 0.000001f;	//add multiplier to make tuning easier
	r     = rpar * 0.001f;		//add multiplier to make tuning easier
	p     = ppar * 0.001f;		//add multiplier to make tuning easier
	x     = intialValue;		//set intial value, can be zero if unknown
	lastX = intialValue;		//set intial value, can be zero if unknown
	k = 0.0f;					//kalman gain,
}
KalmanFilter::~KalmanFilter(){}

/* NullFilter */
float NullFilter::apply(float input){
    return input;
}
