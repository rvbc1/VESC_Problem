/*
 * Buzzer.h
 *
 *  Created on: 15.09.2018
 *      Author: mice
 */

#ifndef CLASSES_BUZZER_H_
#define CLASSES_BUZZER_H_

#include "string.h"
#include "stdint.h"

#include "cmsis_os.h"

class Buzzer {
	char keynotes[12][3] = {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};
	const float base_a4 = 440.f;

	uint16_t mode = 0x00;


	void SetBeepPower(float pwr);
	void SetPWMFrequency(float freq);
	float NoteToFrequency(uint8_t note);

	void Mute(void);
	void Beep(uint8_t power, uint8_t pitch);
	void Tone(char* keynote, uint16_t duration = 0, uint16_t pause = 0);
public:
	enum {
	MUTED = 0,
	LOW_BATTERY = 1 << 1,
	IMU_CALIBRATION = 1 << 2,
	ONE_BEEP = 1 << 3,
	THREE_BEEPS = 1 << 4
	};
	void Init(void);
	void Loop(void);

	void EnableMode(uint16_t m);
	void DisableMode(uint16_t m);

	Buzzer();
	virtual ~Buzzer();
};
extern Buzzer buzzer;
#endif /* CLASSES_BUZZER_H_ */
