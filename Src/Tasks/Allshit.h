/*
 * Allshit.h
 *
 *  Created on: 22.03.2018
 *      Author: mice
 */

#ifndef TASKS_ALLSHIT_H_
#define TASKS_ALLSHIT_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "cmsis_os.h"

void Allshit_begin(void);

extern osThreadId GyroTaskHandle;
extern osThreadId AHRSTaskHandle;
extern osThreadId BatteryManagerHandle;
extern osThreadId SteeringTaskHandle;
extern osThreadId BTTaskHandle;
extern osThreadId FutabaTaskHandle;
extern osThreadId TelemetryTaskHandle;
extern osThreadId USBTaskHandle;
extern osThreadId MotorControllerHandle;
extern osThreadId BuzzerTaskHandle;
extern osThreadId OdometryTaskHandle;

#ifdef __cplusplus
 }
#endif

#endif /* TASKS_ALLSHIT_H_ */
