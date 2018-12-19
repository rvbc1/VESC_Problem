/*
 * PowerManager.cpp
 *
 *  Created on: 21.01.2018
 *      Author: mice
 */

#include <PowerManager.h>
#include "Allshit.h"
#include "stm32f7xx_hal.h"
#include "adc.h"
#include "tim.h"

#include "Buzzer.h"

PowerManager powermanager(4, 4000);
PowerManager::PowerManager(uint8_t cells, uint16_t mAhs) :
		charged_voltage(cells * 4.2f), nominal_voltage(cells * 3.7f), capacity(mAhs)
{

	dT = 0.001f;
	vref = 3.0f;
	resolution  = 0xFFF;

	consumption = 0;
	amp_coeff = 3.333f;

	volt_divider = 11.f;
	volt_finer = 1.015f;
}
void PowerManager::Init(void){

	/* Fucking STM32 device limitation error */
	__HAL_RCC_DAC_CLK_ENABLE();
	/* Due to Errata "STM32F74xxx STM32F75xxx Errata sheet", page 7 (2.2.1):
	* For TIM2/TIM4/TIM5/TIM6/ TRGO or TGRO2
	* events: enable the DAC peripheral clock
	* in the RCC_APB1ENR register
	*/

	MX_ADC1_Init();
	MX_TIM6_Init();

	amp_filter = new BiquadFilter(FILTER_LPF, dT, 50.f);
	volt_filter = new BiquadFilter(FILTER_LPF, dT, 50.f);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_raw, 4);
	HAL_TIM_Base_Start(&htim6);
}

void PowerManager::TaskNotify(void){
	osSignalSet(BatteryManagerHandle, Signal_ADC);
}
void PowerManager::Handler(void) {
	osEvent evt = osSignalWait(Signal_ADC, osWaitForever);
	UNUSED(evt);

	Measure();
//	Calculate();
	Smooth();
	battery_cells = (uint8_t) ceilf(voltage / 4.25f);

	if (battery_cells < 2) // POWERED FROM USB
			{
	} else if (voltage < battery_cells * 3.5f)
		buzzer.EnableMode(Buzzer::LOW_BATTERY);
	else
		buzzer.DisableMode(Buzzer::LOW_BATTERY);
}
void PowerManager::Measure(void){
	Volts_raw = (float)adc_raw[0] * vref / resolution * volt_divider * volt_finer;
	Amps_raw = (float)adc_raw[1] * vref / resolution * amp_coeff;
	AN_IN_raw = (float)adc_raw[2]* vref / resolution;
	Temp_raw = ((float)adc_raw[3]* vref / resolution - 0.76f) / 0.0025f + 25.f;
}
void PowerManager::Calculate(void){
	consumption += Amps_raw * dT / 3.6f;

	fuel_left = capacity - consumption;
	percentage = 100.f* fuel_left / capacity;
}
void PowerManager::Smooth(void){
	amperage = amp_filter->apply(Amps_raw);
	voltage = volt_filter->apply(Volts_raw);
	temperature = Temp_raw;
	analog_in = AN_IN_raw;
}
uint8_t PowerManager::GetBatteryCells(void){
	return battery_cells;
}
PowerManager::~PowerManager() {
	// TODO Auto-generated destructor stub
}

