/*
 * USBTask.h
 *
 *  Created on: 14.04.2018
 *      Author: mice
 */

#ifndef USB_USBTASK_H_
#define USB_USBTASK_H_

#ifdef __cplusplus
 extern "C" {
#endif


extern float motor_logging0[8000];
extern float motor_logging1[8000];
extern float motor_logging2[8000];

extern int32_t USB_TX_signal;

void USB_Init(void);
void USB_Process(void);

typedef struct odroid_setpoints_s{
	float fi;
	float dfi;
	float velocity;
	float acceleration;
	float jerk;
} odroid_setpoints_t;
extern odroid_setpoints_t odroid_setpoints;
#ifdef __cplusplus
 }
#endif
#endif /* USB_USBTASK_H_ */
