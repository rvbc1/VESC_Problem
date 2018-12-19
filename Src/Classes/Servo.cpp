/*
 * Servo.cpp
 *
 *  Created on: 17.09.2018
 *      Author: mice
 */

#include <Servo.h>
#include "Tools.h"
#include "Mathematics.h"

#include "tim.h"

Servo servo(1500, 500, 45.f);

void Servo::Init(void){
	MX_TIM2_Init();
	SetPWM(pwm_middle);
}

void Servo::Disarm(void) {
	pwm_last = GetPWM();
	SetPWM(0);
	if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) == HAL_OK)
		tim_running = 0;
}
void Servo::Arm(void) {
	if (!tim_running) {
		SetPWM(pwm_last);
		if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) == HAL_OK)
			tim_running = 1;
	}
}
void Servo::SetAngleD(float angle, float velocity) {
	angle = constrainf(angle, -max_degrees, max_degrees);
	set_angle = angle;
	set_velocity = velocity;
}
void Servo::SetAngleR(float angle, float velocity) {
	angle = constrainf(angle, -max_radians, max_radians);
	set_angle = RADIANS_TO_DEGREES(angle);
	set_velocity = RADIANS_TO_DEGREES(velocity);
}
void Servo::PositionTracking(void) {
	Arm();
	int32_t now = tools.GetMicros();
	static int32_t before = now;
	if (set_velocity) {
		float dt = (now - before) * 1e-6F;
		current_angle += SIGNF(set_angle - current_angle) * set_velocity * dt;
		constrainf(current_angle, -max_degrees, max_degrees);

	} else {
		current_angle = set_angle;
	}
	SetPWM(current_angle / max_degrees * pwm_band + pwm_middle);
	before = now;
}

void Servo::SetPWM(uint16_t value){
	TIM2->CCR4 = value;
}
uint16_t Servo::GetPWM(void){
	return TIM2->CCR4;
}
Servo::Servo(uint16_t middle, uint16_t band, float angle): pwm_middle(middle), pwm_band(band), max_degrees(angle) {
	pwm_last = pwm_middle;
	max_radians = DEGREES_TO_RADIANS(max_degrees);
}

Servo::~Servo() {
	// TODO Auto-generated destructor stub
}

