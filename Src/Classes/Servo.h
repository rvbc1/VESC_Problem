/*
 * Servo.h
 *
 *  Created on: 17.09.2018
 *      Author: mice
 */

#ifndef CLASSES_SERVO_H_
#define CLASSES_SERVO_H_

#include "stdint.h"

class Servo {
	uint16_t pwm_middle;
	uint16_t pwm_band;
	uint16_t pwm_last;

	float max_degrees;
	float max_radians;


	uint8_t tim_running = 0;
	void SetPWM(uint16_t value);
	uint16_t GetPWM(void);


	float current_angle = 0.f;
	float set_angle = 0.f;
	float set_velocity = 0.f;
public:
	void Init(void);
	void PositionTracking(void);

	void Disarm(void);
	void Arm(void);
	void SetAngleD(float angle = 0.f, float velocity = 0.f);
	void SetAngleR(float angle = 0.f, float velocity = 0.f);
	Servo(uint16_t middle = 1500, uint16_t band = 500, float angle = 45.f);
	virtual ~Servo();
};
extern Servo servo;
#endif /* CLASSES_SERVO_H_ */
