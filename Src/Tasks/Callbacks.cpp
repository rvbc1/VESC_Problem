/*
 * Callbacks.cpp
 *
 *  Created on: 17.09.2018
 *      Author: mice
 */

#include "Allshit.h"

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"

#include "Bluetooth.h"
#include "Futaba.h"
#include "Telemetry.h"
#include "Gyro.h"
#include "PowerManager.h"

/* INTERTUPT CALLBACKS */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART8)
	{
		osSignalSet(BTTaskHandle, 0x01);
	}

	if (huart->Instance == USART3) //aparatura
	{
		futaba.RxCallback();
	}
	if (huart->Instance == huart4.Instance) {
		telemetry.RxCallback();
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART8)
	{
		osSignalSet(BTTaskHandle, 0x02);
	}
	if (huart->Instance == huart4.Instance) {
		telemetry.TxCallback();
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Gyro Data READY
	if (GPIO_Pin == MPU_DRDY_Pin)
		gyro.DataReady();
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	// Completed Gyro Transmission
	if (hspi->Instance == hspi1.Instance)
		gyro.TxCpltCallback();
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	// Completed Gyro Receive
	if (hspi->Instance == hspi1.Instance) {
		gyro.RxCpltCallback();
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == hadc1.Instance) {
		powermanager.TaskNotify();
	}
}

