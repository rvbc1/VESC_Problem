/*
 * Bluetooth.cpp
 *
 *  Created on: 17.09.2018
 *      Author: mice
 */

#include "Allshit.h"

#include "stdint.h"

#include "usart.h"
#include "PowerManager.h"
#include "Motor.h"
#include "Futaba.h"
#include "Gyro.h"
#include "USBTask.h"
#include "AHRS.h"

#include "string.h"

uint8_t terminal_tx[512];
uint8_t terminal_rx[8];
uint16_t terminal_length = 0;

void Bluetooth_Init(void) {
	MX_UART8_Init();

	HAL_UART_Receive_IT(&huart8, terminal_rx, 1);
	osDelay(500);
}
void Bluetooth_Process(void) {
	osEvent signal_type = osSignalWait(0x01 | 0x02, osWaitForever);
	if (signal_type.value.signals == 0x01) {
		switch (terminal_rx[0]) {

		case 's':
			terminal_length = sprintf((char *) terminal_tx, "Task:\t\t\t\tTick:\t\tRun Time %%\n");
			vTaskGetRunTimeStats((char*) &terminal_tx[terminal_length]);
			terminal_length = strlen((char*) terminal_tx);
			terminal_length += sprintf((char *) terminal_tx+terminal_length, "-------\n");
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		case 'B':
			terminal_length = sprintf((char*) terminal_tx, "\nLi-Po:\t%.2fV\nCurrent:\t%.3fA\nAN_IN:\t%.2fV\nSTM Temperature:%.2fC\n", powermanager.voltage,
					powermanager.amperage, powermanager.analog_in, powermanager.temperature);
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		case 'e':
			terminal_length = sprintf((char *) terminal_tx, "Current spd: %.1f. Setspeed: %.1f\nTotalCount: %d, TotalRoad: %.1f\n", motor.getVelocity(),
					motor.getSetVelocity(), motor.getImpulses(), motor.getDistance());
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		case 'f':
			terminal_length = sprintf((char *) terminal_tx,
					"A: %d\tE: %d\tT: %d\tR: %d\nSwitchA: %d\tSwitchB: %d\nSwitchC: %d\tSwitchD:%d\tSwitchE:%d\tSwitchF: %d\n", futaba.sbusChannelData[0],
					futaba.sbusChannelData[1], futaba.sbusChannelData[2], futaba.sbusChannelData[3], futaba.SwitchA, futaba.SwitchB, futaba.SwitchC,
					futaba.SwitchD, futaba.SwitchE, futaba.SwitchF);
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		case 'F':
			terminal_length = sprintf((char *) terminal_tx, "A: %f\tE: %f\tT: %f\tR: %f\n", futaba.StickDeflection[ROLL], futaba.StickDeflection[PITCH],
					futaba.StickDeflection[THROTTLE], futaba.StickDeflection[YAW]);
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		case 'G':
			terminal_length = sprintf((char*) terminal_tx,
					"\nrates:\t%.2f\t%.2f\t%.2f\nangles:\t%.2f\t%.2f\t%.2f\ntemperature:\t%.1fC\naccels:\t%.2fG\t%.2fG\t%.2fG\n", gyro.rates[0], gyro.rates[1],
					gyro.rates[2], ahrs.attitude.values.roll, ahrs.attitude.values.pitch, ahrs.attitude.values.yaw, gyro.temperature, gyro.g_rates[0],
					gyro.g_rates[1], gyro.g_rates[2]);
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		case 'g':
			gyro.StartCalibration();
			terminal_length = sprintf((char*) terminal_tx, "Gyro recalibrating . . .\n\r");
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		case 'r':
			terminal_length = sprintf((char *) terminal_tx, "\n...STM32 Resetting ..\n");
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			NVIC_SystemReset();
			break;
		case 'R':
			terminal_length = sprintf((char *) terminal_tx, "\n...Jumping to System Memory Bootloader ..\n");
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			systemResetToBootloader();
			break;
		case 'u':
			terminal_length = sprintf((char *) terminal_tx, "USBServo: %.1f\tUSBVelocity: %.1f\n", odroid_setpoints.fi, odroid_setpoints.velocity);
			HAL_UART_Transmit_DMA(&huart8, terminal_tx, terminal_length);
			break;
		}
		HAL_UART_Receive_IT(&huart8, terminal_rx, 1);
	} else if (signal_type.value.signals == 0x02) {
	}
}

