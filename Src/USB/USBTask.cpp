/*
 * USBTask.c
 *
 *  Created on: 14.04.2018
 *      Author: mice
 */
#ifdef __cplusplus
 extern "C" {
#endif

#include "Allshit.h"
#include "USBTask.h"

#include "main.h"
#include "stm32f7xx_hal.h"

#include "usbd_cdc_if.h"
#include "string.h"
#include "stdbool.h"

#include "Futaba.h"
#include "Telemetry.h"
#include "Motor.h"
#include "Gyro.h"
#include "Tools.h"
#include "AHRS.h"
#include "PowerManager.h"
#include "Buzzer.h"
#include "Odometry.h"

extern void MX_USB_DEVICE_Init(void);

void USB_Receive_Data(void);
void USB_Transmit_Data(void);
void USB_SetTransmission(void);
void USB_ProcessCommand(void);

void TerminalFn();

extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t usbTxBuffer[1024];
uint32_t len;
uint8_t* usbRxBuffer;
uint32_t usbBytesRead = 0;


int32_t USB_RX_signal = 1 << 0;
int32_t USB_TX_signal = 1 << 1;
uint8_t usbDenominator = 5;


odroid_setpoints_t odroid_setpoints = {0.f, 0.f, 0.f, 0.f, 0.f};


uint8_t usbDATA[256];
uint8_t able_to_receive = 0;

//float motor_logging0[8000];
//float motor_logging1[8000];
//float motor_logging2[8000];

static bool CommunicationOnGoing = false;

#define USB_RXFRAME_SIZE 18
void USB_Init(void) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
}
void USB_Process(void) {
	/* start -> code -> length -> DATA[length] -> length -> code -> stop */
	/* 6 + length */
	osEvent evt = osSignalWait(0, 500);
	if (evt.status == osEventSignal) {
		if (evt.value.signals & USB_TX_signal && CommunicationOnGoing && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
			static uint8_t cnt = 0;
			if (++cnt >= usbDenominator) {
				cnt = 0;
				USB_Transmit_Data();
			}
		}
		if (evt.value.signals & USB_RX_signal) {
			if (usbBytesRead == USB_RXFRAME_SIZE) {
				/* DATA */
				USB_Receive_Data();
			} else if (usbBytesRead == 4) {
				/* SETTINGS */
				USB_SetTransmission();
			} else if (usbBytesRead == 3) {
				/* COMMAND */
				USB_ProcessCommand();
			} else if (usbBytesRead == 1) {
				/* Terminal */
				TerminalFn();
			}
			usbBytesRead = 0;
		}
//			static uint16_t cnt = 0;
//			if (cnt++ > 500) {
//				HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
//				cnt = 0;
//			}
	} else if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
//			HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
	}
}

/* DATA FRAME - code: 0x40 */
struct UsbRxFrame_s {
    uint8_t startByte;
    uint8_t code;
    uint8_t length;

    uint32_t timecode;

    int16_t steering_fi;
    int16_t steering_dfi;

    int16_t speed;
    int16_t acceleration;
    int16_t jerk;

    uint8_t endByte;
} __attribute__ ((__packed__));

union UsbRxFrame_u {
	uint8_t* bytes;
	struct UsbRxFrame_s* frame;
}UsbRxFrame;

/* COMMAND FRAME - codes:
 *
 * 0x01 - DATA enable
 * 0x02 - DATA disable
 *
 * 0x10 - Recalibration
 *
 * 0x20 - Idle
 *
 * 0x30 - Ready
 *
 * 0xDE - Bootloader
 * 0xAD - Reset
 */

/* TRANSMISSION SETTING - code 0xBE
 * STARTBYTE -> CODE -> DATA -> ENDBYTE
 * 0xFF -> 0xBE -> 0x -> 0xFE
 * DATA is 1 byte uint8_t DATA TRANSMISSION LOOPTIME IN MS;
 */


void USB_Receive_Data(void) {
	UsbRxFrame.bytes = usbRxBuffer;
	if (UsbRxFrame.frame->startByte == 0xff && UsbRxFrame.frame->code == 0x40 && UsbRxFrame.frame->length == USB_RXFRAME_SIZE - 4
			&& UsbRxFrame.frame->endByte == 0xfe) {

		/* Transmission rate Test */
		static uint8_t cnt = 0;
		static uint32_t old_timecode = UsbRxFrame.frame->timecode;
		if (UsbRxFrame.frame->timecode != old_timecode) {
			if (++cnt >= 50) {
				HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
				cnt = 0;
			}
		} else
			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
		old_timecode = UsbRxFrame.frame->timecode;

		// potrzebny w¹tek na servo i silnik uwzglêdniaj¹cy pochodne sterowania pomiêdzy instrukcjami.
		if (UsbRxFrame.frame->steering_fi > 10000.f * M_PI_FLOAT / 4)
			UsbRxFrame.frame->steering_fi = 10000.f * M_PI_FLOAT / 4;
		else if (UsbRxFrame.frame->steering_fi < - 10000.f *M_PI_FLOAT / 4)
			UsbRxFrame.frame->steering_fi = - 10000.f * M_PI_FLOAT / 4;

		odroid_setpoints.fi = (float)(UsbRxFrame.frame->steering_fi * 18.f / 1000.f / M_PI_FLOAT);
		odroid_setpoints.dfi = (float)(UsbRxFrame.frame->steering_dfi * 18.f / 1000.f / M_PI_FLOAT);

		odroid_setpoints.velocity = (float)(UsbRxFrame.frame->speed);
		odroid_setpoints.acceleration = (float)(UsbRxFrame.frame->acceleration);
		odroid_setpoints.jerk = (float)(UsbRxFrame.frame->jerk);
	}
}

#define USB_TXFRAME_SIZE 36
struct UsbOFrame_s {
    uint8_t startbyte;
    uint8_t code;
    uint8_t length;

    uint32_t timecode;

    int32_t distance;
    int16_t velocity;
    int16_t w, x, y, z;
    uint16_t yaw;
    int16_t rates[3];
    int16_t acc[3];

    uint8_t endByte;
} __attribute__ ((__packed__));

union UsbOFrame_u {
    uint8_t bytes[USB_TXFRAME_SIZE];
    struct UsbOFrame_s frame;
} usbOframe;

void USB_Transmit_Data(void){
	usbOframe.frame.startbyte = 0xFF;
	usbOframe.frame.code = 0x40;
	usbOframe.frame.length = USB_TXFRAME_SIZE-4;

	usbOframe.frame.timecode = HAL_GetTick();

	usbOframe.frame.distance = motor.getDistance();
	usbOframe.frame.velocity = motor.getVelocity();

	quaternion orientation;
	ahrs.getQuaternion(&orientation);
	usbOframe.frame.w = (int16_t)(orientation.w * 32767.f);
	usbOframe.frame.x = (int16_t)(orientation.x * 32767.f);
	usbOframe.frame.y = (int16_t)(orientation.y * 32767.f);
	usbOframe.frame.z = (int16_t)(orientation.z * 32767.f);

	usbOframe.frame.yaw = (uint16_t)(gyro.angles[2]/360.f*65536.f);

	for(uint8_t axis = 0; axis < 3; axis++){
	usbOframe.frame.rates[axis] = (int16_t)(gyro.rates[axis] * gyro.getGyroScale());
	usbOframe.frame.acc[axis] = (int16_t)(gyro.g_rates[axis] * gyro.getAccScale());
	}

	usbOframe.frame.endByte = 0xFE;

	CDC_Transmit_FS(usbOframe.bytes, USB_TXFRAME_SIZE);
}
void USB_SetTransmission(void)
{
	if (usbRxBuffer[0] == 0xff && usbRxBuffer[1] == 0xBE && usbRxBuffer[3] == 0xfe) {
		usbDenominator = usbRxBuffer[2];
	}
}
void USB_ProcessCommand(void){
	if (usbRxBuffer[0] == 0xff && usbRxBuffer[2] == 0xfe) {
		switch (usbRxBuffer[1]) {

		case 0x01:
			CommunicationOnGoing = true;
			break;
		case 0x02:
			CommunicationOnGoing = false;
			break;
		case 0x10:
			gyro.StartCalibration();
			odometry.Reset(ahrs.attitude.values.yaw, motor.getDistance(),tools.GetMicros());
			odometry.SetCurrentPosition();
			break;
		case 0x20:

			break;
		case 0x30:

			break;
		case 0xDE:
			systemResetToBootloader();
			break;
		case 0xAD:
			NVIC_SystemReset();
			break;
		}
	}
}
int8_t MAIN_USB_Receive(uint8_t* Buf, uint32_t *Len) {
	usbRxBuffer = Buf;
	usbBytesRead = *Len;
	osSignalSet(USBTaskHandle, USB_RX_signal);
	return 0;
}
void TerminalFn(){
	switch (usbRxBuffer[0]) {

	case 's': {
		len = sprintf((char *) usbTxBuffer, "Task:\t\t\t\tTick:\t\tRun Time %%\n");
		vTaskGetRunTimeStats((char*) usbTxBuffer+len);
		len = strlen((char*) usbTxBuffer);
		len += sprintf((char *) usbTxBuffer+len, "-------\n");
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	}
	case 'B':
		len = sprintf((char*) usbTxBuffer, "\nLi-Po:\t%.2fV\nCurrent:\t%.3fA\nAN_IN:\t%.2fV\nSTM Temperature:%.2fC\n", powermanager.voltage,
				powermanager.amperage, powermanager.analog_in, powermanager.temperature);
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'e':
		len = sprintf((char *) usbTxBuffer, "Current spd: %.1f\tSetspeed: %.1f\nTotalCount: %ld\tTotalRoad: %.1f\n\n", motor.getVelocity(), motor.getSetVelocity(), motor.getImpulses(), motor.getDistance());
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'f':
		len = sprintf((char *) usbTxBuffer, "A: %d\tE: %d\tT: %d\tR: %d\nSwitchA: %d\tSwitchB: %d\n", futaba.sbusChannelData[0], futaba.sbusChannelData[1],
				futaba.sbusChannelData[2], futaba.sbusChannelData[3], futaba.SwitchA, futaba.SwitchB);
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'F':
		len = sprintf((char *) usbTxBuffer, "A: %f\tE: %f\tT: %f\tR: %f\n", futaba.StickDeflection[ROLL], futaba.StickDeflection[PITCH],
				futaba.StickDeflection[THROTTLE], futaba.StickDeflection[YAW]);
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'G':
		len = sprintf((char*) usbTxBuffer, "\nrates:\t%.2f\t%.2f\t%.2f\nangles:\t%.2f\t%.2f\t%.2f\tFLAT: %.2f\ntemperature:\t%.1fC\naccels:\t%.2fG\t%.2fG\t%.2fG\n",
				gyro.rates[0], gyro.rates[1], gyro.rates[2], ahrs.attitude.values.roll, ahrs.attitude.values.pitch, ahrs.attitude.values.yaw, gyro.angles[2], gyro.temperature,
				gyro.g_rates[0], gyro.g_rates[1], gyro.g_rates[2]);
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'q':
		quaternion orientation;
		ahrs.getQuaternion(&orientation);
		len = sprintf((char*) usbTxBuffer, "\nw = %.3f\tx = %.3f\ty = %.3f\tz = %.3f\n",
				orientation.w, orientation.x, orientation.y, orientation.z);
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'g':
		gyro.StartCalibration();
		odometry.Reset(ahrs.attitude.values.yaw, motor.getDistance(),tools.GetMicros());
		odometry.SetCurrentPosition();
		len = sprintf((char*) usbTxBuffer, "Gyro recalibrating . . .\n\r");
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'M':
		len = sprintf((char *) usbTxBuffer, "velocity: %.1f\ndistance: %.1f\n\n", motor.getVelocity(), motor.getDistance());
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'o':
		len = sprintf((char *) usbTxBuffer, "x: %.1f\ty: %.1f\nVx: %.1f\tVy: %.1f\n\n", odometry.getX(), odometry.getY(), odometry.getVx(), odometry.getVy());
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'O':
		TIM3->CNT += -1500;
		len = sprintf((char *) usbTxBuffer, "dist: %.1f\n\n", motor.getDistance());
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'm':
//						len = sprintf((char *) usbTxBuffer, "time:\tset_pwm:\trpms:\n");
//						for(int index = 0; index < 8000; index = index + 8) {
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index], motor_logging1[index], motor_logging2[index]);
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index+1], motor_logging1[index+1], motor_logging2[index+1]);
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index+2], motor_logging1[index+2], motor_logging2[index+2]);
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index+3], motor_logging1[index+3], motor_logging2[index+3]);
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index+4], motor_logging1[index+4], motor_logging2[index+4]);
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index+5], motor_logging1[index+5], motor_logging2[index+5]);
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index+6], motor_logging1[index+6], motor_logging2[index+6]);
//							len += sprintf((char *) usbTxBuffer + len, "%.6f\t%.1f\t%.2f\n", motor_logging0[index+7], motor_logging1[index+7], motor_logging2[index+7]);
//							CDC_Transmit_FS(usbTxBuffer, len);
//							len = 0;
//							osDelay(1);
//						}
		break;
	case 'T':
		CommunicationOnGoing = !CommunicationOnGoing;
		len = sprintf((char*) usbTxBuffer, "Switched to Terminal or ROS Mode\n");
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 't':
		len = sprintf((char*) usbTxBuffer, "time: %lums / %luus\n",HAL_GetTick(), tools.GetMicros());
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'u':
		len = sprintf((char *) usbTxBuffer, "USBServo: %.1f\tUSBVelocity: %.1f\n", odroid_setpoints.fi, odroid_setpoints.velocity);
		CDC_Transmit_FS(usbTxBuffer, len);
		break;
	case 'r':
		len = sprintf((char *) usbTxBuffer, "\n...STM32 Resetting ..\n");
		CDC_Transmit_FS(usbTxBuffer, len);
		NVIC_SystemReset();
		break;
	case 'R':
		len = sprintf((char *) usbTxBuffer, "\n...Jumping to System Memory Bootloader ..\n");
		CDC_Transmit_FS(usbTxBuffer, len);
		systemResetToBootloader();
		break;
	}
}
#ifdef __cplusplus
 }
#endif

