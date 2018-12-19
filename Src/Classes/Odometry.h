/*
 * Odometry.h
 *
 *  Created on: 20.09.2018
 *      Author: mice
 */

#ifndef CLASSES_ODOMETRY_H_
#define CLASSES_ODOMETRY_H_

#include "stdint.h"
#include "Filters.h"

class Odometry {
	int32_t LastUs;
	float LastYaw, LastDistance;
	float x, y, Vx, Vy;
	BiquadFilter LPFx {FILTER_LPF, 0.002f, 25.f}, LPFy {FILTER_LPF, 0.002f, 25.f};
public:
	const int32_t SignalReady = 1 << 0;
	void Init(void);
	void Process(float yaw, float distance, int32_t CurrentUs);

	void SetCurrentPosition(float SetX = 0.f, float SetY = 0.f);
	void Reset(float CurrentYaw = 0.f, float CurrentDistance = 0.f, int32_t CurrentUs = 0);

	float getX(void);
	float getY(void);
	float getVx(void);
	float getVy(void);

	Odometry();
	virtual ~Odometry();
};
extern Odometry odometry;
#endif /* CLASSES_ODOMETRY_H_ */
