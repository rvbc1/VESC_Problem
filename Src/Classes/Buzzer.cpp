/*
 * Buzzer.cpp
 *
 *  Created on: 15.09.2018
 *      Author: mice
 */

#include <Buzzer.h>
#include "math.h"
#include "Allshit.h"

#include "tim.h"

#include "string.h"
#include "stdlib.h"
#include "math.h"
#define TIM1_FREQ 1000000

Buzzer buzzer;
void Buzzer::Init(void){
	MX_TIM1_Init();

	SetPWMFrequency(1000.f);
	Mute();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	osDelay(200);


	Tone("A6", 75, 75);
	Tone("A6", 75, 225);
	Tone("A6", 75, 225);
	Tone("F6", 75, 75);
	Tone("A6", 75, 225);
	Tone("C7", 75, 525);
	Tone("C6", 75, 225);
}
void Buzzer::Loop(void) {
	if (mode) {
		if (mode & LOW_BATTERY) {
			Tone("F6", 500, 500);
		}
		if (mode & IMU_CALIBRATION) {
			Tone("B6", 10, 250);
			DisableMode(IMU_CALIBRATION);
		}
		if (mode & ONE_BEEP) {
			Tone("D7", 250, 250);

			DisableMode(ONE_BEEP);
		}
		if (mode & THREE_BEEPS) {
			Tone("G7", 250, 250);
			Tone("G7", 250, 250);
			Tone("G7", 250, 250);

			DisableMode(THREE_BEEPS);
		}

	} else
		Mute();
	osDelay(5);
}
void Buzzer::SetBeepPower(float pwr){
	if(pwr > 1.f)
		pwr = 1.f;
	else if(pwr < 0.f)
		pwr = 0.f;

	TIM1->CCR3 = (uint16_t) (floorf( pwr * TIM1->ARR / 2.f));
}
void Buzzer::SetPWMFrequency(float freq){
	TIM1->CNT = 0u;
	TIM1->ARR = (uint16_t)(roundf(TIM1_FREQ / freq));
}
void Buzzer::Mute(void){
	TIM1->CCR3 = 0;
}
void Buzzer::Beep(uint8_t power, uint8_t pitch){

}
void Buzzer::Tone(char* keynote, uint16_t duration, uint16_t pause) {
	uint8_t octave = 0, note = 0;
	for (uint8_t i = 0; i < 12; i++) {
		if (keynote[0] == keynotes[i][0]) {
			note = i;
			break;
		}
	}

	if (keynote[1] == '#') {
		note++;
		octave = atoi(&keynote[2]);
	} else
		octave = atoi(&keynote[1]);

	float frequency = NoteToFrequency(octave * 12 + note);

	SetPWMFrequency(frequency);
	SetBeepPower(1.f);
	if (duration) {
		osDelay(duration);
		Mute();
	}
	if (pause)
		osDelay(pause);
}

void Buzzer::EnableMode(uint16_t m) {
	mode |= m;
}
void Buzzer::DisableMode(uint16_t m) {
	mode &= ~m;
}
float Buzzer::NoteToFrequency(uint8_t note) {
	if (note <= 119)
		return base_a4*powf(2.f,(note-57)/12.f);
	return -1;
}





Buzzer::Buzzer() {
	// TODO Auto-generated constructor stub

}

Buzzer::~Buzzer() {
	// TODO Auto-generated destructor stub
}

