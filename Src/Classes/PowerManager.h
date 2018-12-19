/*
 * PowerManager.h
 *
 *  Created on: 21.01.2018
 *      Author: mice
 */

#ifndef MULTIROTOR_POWERMANAGER_H_
#define MULTIROTOR_POWERMANAGER_H_

#include "Filters.h"
#include "stdint.h"

class PowerManager {
	float charged_voltage;
	float dT;

	float vref;
	uint16_t resolution;
	float amp_coeff;
	float volt_divider;
	float volt_finer;

	Filter * amp_filter;
	Filter * volt_filter;

	volatile uint16_t adc_raw[4];
	float Amps_raw;
	float Volts_raw;
	float Temp_raw;
	float AN_IN_raw;

	uint8_t battery_cells = 4;

	int32_t Signal_ADC = 1 << 0;


	uint8_t bip_5times = 10;

	void Measure(void);
	void Calculate(void);
	void Smooth(void);

public:

	float nominal_voltage;
	uint16_t capacity;

	float fuel_left;
	float consumption;
	float percentage;

	float amperage;
	float voltage;
	float analog_in;
	float temperature;

	PowerManager(uint8_t cells, uint16_t mAhs);
	void Init(void);
	void TaskNotify(void);
	void Handler(void);
	uint8_t GetBatteryCells(void);
	virtual ~PowerManager();
};
extern PowerManager powermanager;
#endif /* MULTIROTOR_POWERMANAGER_H_ */
