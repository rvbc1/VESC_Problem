/*
 * Odometry.cpp
 *
 *  Created on: 20.09.2018
 *      Author: mice
 */

#include <Odometry.h>

#include "Mathematics.h"

Odometry odometry;
void Odometry::Init(void) {
//	odometry.Reset();
//	odometry.SetCurrentPosition();
}
void Odometry::Process(float CurrentYaw, float CurrentDistance, int32_t CurrentUs) {
	float _dt = (CurrentUs - LastUs)*1e-6f;

	float derivative = CurrentDistance - LastDistance;
	float dx = derivative * sin_approx(DEGREES_TO_RADIANS(0.5f * (CurrentYaw + LastYaw)));
	float dy = derivative * cos_approx(DEGREES_TO_RADIANS(0.5f * (CurrentYaw + LastYaw)));

	x += dx;
	y += dy;
	Vx = LPFx.apply(dx / _dt);
	Vy = LPFy.apply(dy / _dt);

	LastUs = CurrentUs;
	LastDistance = CurrentDistance;
	LastYaw = CurrentYaw;
}
void Odometry::SetCurrentPosition(float SetX, float SetY){
	x = SetX;
	y = SetY;
}
void Odometry::Reset(float CurrentYaw, float CurrentDistance, int32_t CurrentUs) {
	Vx = 0.f;
	Vy = 0.f;
	LastUs = CurrentUs;
	LastDistance = CurrentDistance;
	LastYaw = CurrentYaw;
}
float Odometry::getX(void) {
	return x;
}
float Odometry::getY(void) {
	return y;
}
float Odometry::getVx(void) {
	return Vx;
}
float Odometry::getVy(void) {
	return Vy;
}



Odometry::Odometry() {
	SetCurrentPosition();
	Reset();
}
Odometry::~Odometry() {}

