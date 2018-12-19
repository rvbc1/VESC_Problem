/*
 * Allshit.cpp
 *
 *  Created on: 22.03.2018
 *      Author: mice
 */

#include "cmsis_os.h"
#include "Allshit.h"
#include "USBTask.h"
#include "Futaba.h"
#include "Gyro.h"
#include "PowerManager.h"
#include "AHRS.h"
#include "Motor.h"
#include "Tools.h"
#include "Telemetry.h"
#include "Buzzer.h"
#include "Servo.h"
#include "Bluetooth.h"
#include "Odometry.h"

#include "Mathematics.h"
osThreadId GyroTaskHandle;
osThreadId AHRSTaskHandle;
osThreadId BatteryManagerHandle;
osThreadId SteeringTaskHandle;
osThreadId BTTaskHandle;
osThreadId FutabaTaskHandle;
osThreadId TelemetryTaskHandle;
osThreadId USBTaskHandle;
osThreadId MotorControllerHandle;
osThreadId BuzzerTaskHandle;
osThreadId OdometryTaskHandle;

void StartGyroTask(void const * argument);
void StartAHRSTask(void const * argument);
void StartOdometryTask(void const * argument);
void StartBatteryManager(void const * argument);
void StartSteeringTask(void const * argument);
void StartBTTask(void const * argument);
void StartFutabaTask(void const * argument);
void StartUSBTask(void const * argument);
void StartTelemetryTask(void const * argument);
void StartMotorController(void const * argument);
void StartBuzzerTask(void const * argument);

#include "iwdg.h"
#include "wwdg.h"

#include "usart.h"
#include "bldc_interface.h"
#include "bldc_interface_uart.h"

static void send_packet(unsigned char *data, unsigned int len) {
	HAL_UART_Transmit(&huart7, data, len, 100);
}

// Your init function
void comm_uart_init(void) {


	bldc_interface_uart_init(send_packet);
}

void Allshit_begin(void) {

	/* definition and creation of FutabaTask */
	osThreadDef(FutabaTask, StartFutabaTask, osPriorityHigh, 0, 256);
	FutabaTaskHandle = osThreadCreate(osThread(FutabaTask), NULL);

	/* definition and creation of MotorController */
	osThreadDef(MotorController, StartMotorController, osPriorityHigh, 0, 512);
	MotorControllerHandle = osThreadCreate(osThread(MotorController), NULL);

	/* definition and creation of SteeringTask */
	osThreadDef(SteeringTask, StartSteeringTask, osPriorityHigh, 0, 512);
	SteeringTaskHandle = osThreadCreate(osThread(SteeringTask), NULL);

	/* definition and creation of GyroTask */
	osThreadDef(GyroTask, StartGyroTask, osPriorityHigh, 0, 1024);
	GyroTaskHandle = osThreadCreate(osThread(GyroTask), NULL);

	/* definition and creation of AHRSTask */
	osThreadDef(AHRSTask, StartAHRSTask, osPriorityHigh, 0, 256);
	AHRSTaskHandle = osThreadCreate(osThread(AHRSTask), NULL);

	/* Odometry - HIGH PRIORITY*/
	osThreadDef(OdometryTask, StartOdometryTask, osPriorityHigh, 0, 128);
	OdometryTaskHandle = osThreadCreate(osThread(OdometryTask), NULL);

	/* definition and creation of BatteryManager */
	osThreadDef(BatteryManager, StartBatteryManager, osPriorityBelowNormal, 0, 256);
	BatteryManagerHandle = osThreadCreate(osThread(BatteryManager), NULL);

	/* definition and creation of USBTask */
	osThreadDef(USBTask, StartUSBTask, osPriorityHigh, 0, 256);
	USBTaskHandle = osThreadCreate(osThread(USBTask), NULL);

	/* definition and creation of TelemetryTask */
	osThreadDef(TelemetryTask, StartTelemetryTask, osPriorityNormal, 0, 256);
	TelemetryTaskHandle = osThreadCreate(osThread(TelemetryTask), NULL);

	/* definition and creation of BTTask */
	osThreadDef(BTTask, StartBTTask, osPriorityLow, 0, 256);
	BTTaskHandle = osThreadCreate(osThread(BTTask), NULL);

	/* Buzzer - LOW PRIORITY */
	osThreadDef(BuzzerTask, StartBuzzerTask, osPriorityLow, 0, 128);
	BuzzerTaskHandle = osThreadCreate(osThread(BuzzerTask), NULL);

}

void StartFutabaTask(void const * argument) {
	futaba.Init();
	for (;;) {
		futaba.Process();
	}

}

void StartMotorController(void const * argument) {
	motor.Init();
	while(1) {
		motor.Process();
		osSignalSet(OdometryTaskHandle, odometry.SignalReady);
	}
}

static void StickCommandProccess(void) {
	if (futaba.Stick_Command[1]) // (   .)    (   .)
		motor.SetPassthroughState(true);
	else
		motor.SetPassthroughState(false);
	if (futaba.Stick_Command[0]) // (.   )    (.   )
		motor.setMaxVelocity(6000.f);
	else
		motor.setMaxVelocity(2500.f);

	static bool last_cmd = false;
	if (futaba.Stick_Command[4] != last_cmd) { // (.   )    (   .)   < - to nie sa cycki
		last_cmd = futaba.Stick_Command;
		gyro.StartCalibration();
		odometry.Reset(ahrs.attitude.values.yaw, motor.getDistance(),tools.GetMicros());
		odometry.SetCurrentPosition();
	}
}
void StartSteeringTask(void const * argument) {
	enum {
		DISARMED = 0,
		MODE_AUTONOMOUS,
		MODE_SEMI,
		MODE_ACRO
	} rc_mode = DISARMED;

	const uint32_t task_dt = 1u;

	MX_UART7_Init();
	comm_uart_init();

	futaba.ConfigureSmoothing(50.f, task_dt * 1e-3); /* Nyquist frequency - 1/2 Radio frequency * 0.9; 8CH - 9ms, 16CH - 18ms,*/

	servo.Init();
	osDelay(300);
	for (;;) {
		futaba.ProcessSmoothing();

		if (futaba.Get_RCState() || futaba.SwitchA < SWITCH_DOWN) {
			rc_mode = DISARMED;

			servo.Disarm();
			motor.Disarm();
			StickCommandProccess();
		} else if (futaba.SwitchA == SWITCH_DOWN) {
			if (futaba.SwitchB == SWITCH_UP) {
				rc_mode = MODE_ACRO;

				servo.SetAngleD(futaba.SmoothDeflection[YAW] * 45.f, 720.f);
				motor.SetDuty(futaba.SmoothDeflection[PITCH]);
				motor.SetVelocity(motor.getMaxVelocity() * futaba.SmoothDeflection[PITCH], 10000.f, 40000.f);
				bldc_interface_set_duty_cycle(0.2*futaba.SmoothDeflection[PITCH]);
			} else if (futaba.SwitchB == SWITCH_MIDDLE) {
				rc_mode = MODE_SEMI;

				servo.SetAngleD(odroid_setpoints.fi, odroid_setpoints.dfi);
				motor.SetDuty(futaba.SmoothDeflection[PITCH]);
				motor.SetVelocity(motor.getMaxVelocity() * futaba.SmoothDeflection[PITCH], 10000.f, 40000.f);

				//bldc_interface_set_duty_cycle(0);
			} else if (futaba.SwitchB == SWITCH_DOWN) {
				rc_mode = MODE_AUTONOMOUS;

				servo.SetAngleD(odroid_setpoints.fi, odroid_setpoints.dfi);
				motor.SetVelocity(odroid_setpoints.velocity, odroid_setpoints.acceleration, odroid_setpoints.jerk);
			}

			servo.PositionTracking();
			motor.Arm();
		}

		//TODO - Find best suited place for watchdog refreshes
		static uint8_t watchdog_init_done = 0;
		if (watchdog_init_done) {
			HAL_WWDG_Refresh(&hwwdg);
			HAL_IWDG_Refresh(&hiwdg);
		} else {
			MX_IWDG_Init();
			MX_WWDG_Init();
			watchdog_init_done = 1;
		}

		osDelay(task_dt);
	}
	UNUSED(rc_mode);
}

void StartGyroTask(void const * argument) {
	gyro.Init();
	while(1) {
		gyro.Process();
	}
}

void StartAHRSTask(void const * argument) {
	ahrs.Init();
	while(1) {
		ahrs.Process();
		osSignalSet(USBTaskHandle, USB_TX_signal);
	}
}


void StartOdometryTask(void const * argument) {
	odometry.Init();
	while (1) {
		osSignalWait(odometry.SignalReady, osWaitForever);
		odometry.Process(ahrs.attitude.values.yaw, motor.getDistance(), tools.GetMicros());
	}
}

void StartBatteryManager(void const * argument) {
	powermanager.Init();
	for (;;) {
		powermanager.Handler();
	}

}

void StartUSBTask(void const * argument) {
	USB_Init();
	while(1)
	{
		USB_Process();
	}
}

void StartTelemetryTask(void const * argument) {
	telemetry.Init();
	for (;;) {
		telemetry.Process();
	}

}

void StartBTTask(void const * argument) {
	Bluetooth_Init();
	for (;;) {
		Bluetooth_Process();
	}
}

void StartBuzzerTask(void const * argument){
	buzzer.Init();
	while(1)
	{
		buzzer.Loop();
	}
}
