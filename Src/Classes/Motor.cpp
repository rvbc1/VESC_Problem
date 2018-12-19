/*
 * Motor.cpp
 *
 *  Created on: 22.06.2018
 *      Author: mice
 */

#include <Motor.h>
#include "Allshit.h"
#include "PowerManager.h"
#include "Tools.h"
#include "tim.h"


Motor motor(1500, 500);



void Motor::Init(){
	MX_TIM3_Init();
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	MX_TIM4_Init();
	SetPWM(pwm_middle);
	SetPassthroughState(true);
	SetDuty(0.f);
	Arm();
	//osDelay(200);
	Disarm();
	SetPassthroughState(true);
}
void Motor::Process(void) {
	Read();
	Conversions();
	SpeedTracking();
	Controller();
	//if (tim_running)
	Output();

	osDelay(_dt * 1000.f);
}
void Motor::Read(void){
	static int16_t oldCount = 0;
	int16_t count = TIM3->CNT;
	impulses = - (count - oldCount);
	oldCount = count;
	totalImpulses += impulses;
}
void Motor::Conversions(void) {
	float filtered_impulses = lpf.apply(impulses);

	current_rpm = filtered_impulses * enc_to_rpm;
	rotations = totalImpulses * enc_to_rotations;

	current_velocity = filtered_impulses * enc_to_mms;
	distance = totalImpulses * enc_to_mm;


}
void Motor::Controller(void){

	static float previous_error = 0;

	int32_t now = tools.GetMicros();
	static int32_t before = now;
	float dt = constrainf((now - before) * 1e-6F, (_dt/2), (_dt*2));

	before = now;

	set_rpm = current_set_velocity / rpm_to_mms;

	float setpoint = current_set_velocity;
	float measurement = current_velocity;


	float error = setpoint - measurement;

	Proportional = Kp * error;

	Integral += Ki *error * dt;
	Integral = constrainf(Integral, -windup_limit, windup_limit);

	Derivative = Kd * dterm_lpf.apply(error - previous_error) / dt;
	previous_error = error;

	if (controller_en) {
		float vBatScaling = 1.f;
		if (powermanager.voltage > 6.f)
			vBatScaling = 8.4f / powermanager.voltage;
		pid_value = vBatScaling * (Proportional + Integral + Derivative);
		pid_value += SIGNF(pid_value) * duty_deadband;
		pid_value = constrainf(pid_value, -1.f, 1.f);
	} else {
		previous_error = 0.f;
		Integral = 0.f;
		pid_value = 0.f;
	}
}
void Motor::Output(void) {
	if (passthrough) {
//		if(set_duty > 0.25 || set_duty < 0){
//			bldc_interface_set_duty_cycle(0.02);
//		}
			//bldc_interface_set_duty_cycle(set_duty);
		SetPWM(set_duty * pwm_band + pwm_middle);
	} else{
		//bldc_interface_set_duty_cycle(set_duty);
		SetPWM(pid_value * pwm_band + pwm_middle);
//		if(set_duty > 0.25 || set_duty < 0){
//			bldc_interface_set_duty_cycle(0.02);
//		}
			//bldc_interface_set_duty_cycle(set_duty);
	}
}
void Motor::Arm(void) {
	controller_en = true;
	if (!tim_running) {
		SetPWM (pwm_last);
		if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) == HAL_OK)
			tim_running = 1;
	}
}
void Motor::Disarm(void) {
	controller_en = false;
	pwm_last = GetPWM();
	SetPWM(0);
	if (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4) == HAL_OK)
		tim_running = 0;
}
float Motor::getRPMs(void){
	return current_rpm;
}
float Motor::getVelocity(void){
	return current_velocity;
}
float Motor::getSetVelocity(void){
	return current_set_velocity;
}
float Motor::getDistance(void){
	return distance;
}
int32_t Motor::getImpulses(void){
	return totalImpulses;
}
float Motor::getMaxVelocity(void){
	return max_velocity;
}
void Motor::setMaxVelocity(float velocity){
	max_velocity = velocity;
}
void Motor::SpeedTracking(void) {
	//	Arm();
	int32_t now = tools.GetMicros();
	static int32_t before = now;
	if (set_acceleration) {
		float dt = (now - before) * 1e-6F;

		if (set_jerk) {
			current_acceleration += SIGNF(set_acceleration - current_acceleration) * set_jerk * dt;
			constrainf(current_acceleration, -max_acceleration, max_acceleration);
		} else {
			current_acceleration = set_acceleration;
		}

		current_set_velocity += SIGNF(set_velocity - current_set_velocity) * set_acceleration * dt;
		constrainf(current_set_velocity, -max_velocity, max_velocity);

	} else {
		current_set_velocity = set_velocity;
	}
	before = now;
}
void Motor::SetVelocity(float velocity, float acceleration, float jerk) {
	set_velocity = constrainf(velocity, -max_velocity, max_velocity);
	set_acceleration = constrainf(acceleration, -max_acceleration, max_acceleration);
	set_jerk = jerk;
}
void Motor::SetDuty(float duty) {
	set_duty = constrainf(duty, -1.f, 1.f);
}
void Motor::SetPWM(uint16_t value){
	TIM4->CCR4 = value;
}
uint16_t Motor::GetPWM(void){
	return TIM4->CCR4;
}
void Motor::SetControllerState(bool is_enabled){
	controller_en = is_enabled;
}
void Motor::SetPassthroughState(bool is_passthrough){
	passthrough = is_passthrough;
}
Motor::Motor(uint16_t middle, uint16_t band): pwm_middle(middle), pwm_band(band) {
	pwm_last = pwm_middle;
}
Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

