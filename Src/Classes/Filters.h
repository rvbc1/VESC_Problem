/*
 * Filters.h
 *
 *  Created on: 03.01.2018
 *      Author: mice
 */

#ifndef MULTIROTOR_FILTERS_H_
#define MULTIROTOR_FILTERS_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "stm32f7xx_hal.h"
#include "Mathematics.h"

class Filter {

public:
	virtual float apply(float input) = 0;

	Filter(){};
	virtual ~Filter(){};
};

class PT1Filter:
		public Filter {
    float state;
    float k;
public:
	float apply(float input);
	void init(void);

	PT1Filter(uint8_t f_cut, float dT);
	virtual ~PT1Filter();
};

typedef enum {
    FILTER_LPF = 0,
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;
typedef enum {
    LPF_PT1 = 0,
	LPF_BIQUAD,
} LPFFilterType_e;
class BiquadFilter:
		public Filter {

	/* bandwidth in octaves */
const float BIQUAD_BANDWIDTH = 1.9f;
	/* quality factor - butterworth*/
const float BIQUAD_Q = 1.0f / sqrtf(2.0f);

	/* coefficients*/
    float b0, b1, b2, a1, a2;
    /* samples*/
    float x1, x2, y1, y2;
public:
	float apply(float input);
	void init(void);

	BiquadFilter( biquadFilterType_e type, float dT, float filterFreq,  float cutoff = 0);
	virtual ~BiquadFilter();
};
class KalmanFilter:
		public Filter {

    float q;       //process noise covariance
    float r;       //measurement noise covariance
    float p;       //estimation error covariance matrix
    float k;       //kalman gain
    float x;       //state
    float lastX;   //previous state

public:
	float apply(float input);
	void init(void);

	KalmanFilter(float qpar, float rpar, float ppar, float intialValue);
	virtual ~KalmanFilter();
};
class NullFilter:
		public Filter {
public:
	float apply(float input);
	void init(void);

	NullFilter(){};
	virtual ~NullFilter(){};
};
#endif /* MULTIROTOR_FILTERS_H_ */
