/*
 * Gyro.cpp
 *
 *  Created on: 02.01.2018
 *      Author: mice
 */
#include "Gyro.h"

#include "Allshit.h"
#include "AHRS.h"
#include "axis.h"

#include "stm32f7xx_hal.h"
#include "spi.h"

#include "Buzzer.h"

Gyro gyro;

#define YAW_PRECISION 0.0001f
#define YAW_INVPRECISION 10000.f
Gyro::Gyro() {
	if(GYRO_32khz)
		dT = 0.00003125f;
}
void Gyro::Init(void) {
	filters();
	mpu_init();
	StartCalibration();
}
void Gyro::Process(void) {
	static bool even_once = false;

	osEvent evt = osSignalWait(Signal_DR, 5);
	if (evt.status == osEventSignal || even_once) {
		if (evt.status == osEventTimeout) {
			++timeouts;
			if (timeouts > 50)
				Init();
		} else {
			timeouts = 0;
			even_once = true;

			static uint8_t acc_cycling = 0;
			if (acc_cycling)
				acc_cycling--;
			else {
				read_acc = true;
				acc_cycling = acc_denom - 1;
			}

			data_read();
			data_conv();
			calfunction();
			zeroing();
			filtering();
			accumulations();
			if(read_acc)
				ahrs.TaskNotify();
			read_acc = false;

			if (flash_led_3times) {
				static int counter = 0;
				if (counter++ >= 8000/7) {
					HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
					counter = 0;
					flash_led_3times--;
				}
			} else if (!calibration_mode) {
				static int counter = 0;
				if (counter++ >= 8000/2) {
					HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
					counter = 0;
				}
			}
		}
	}
}
HAL_StatusTypeDef Gyro::data_read(void) {
	uint8_t status = HAL_OK;
	if (read_acc) {
		status |= read(MPU_RA_ACCEL_XOUT_H | 0x80, data_buffer, 14);
	} else {
		status |= read(MPU_RA_GYRO_XOUT_H | 0x80, data_buffer+8, 6);
	}

	return (HAL_StatusTypeDef) status;
}
HAL_StatusTypeDef Gyro::data_conv(void) {

	gyro_raw[0] = -(data_buffer[10] << 8 | data_buffer[11]); //X - PITCH
	gyro_raw[1] = (data_buffer[8] << 8 | data_buffer[9]); //Y - ROLL
	gyro_raw[2] = data_buffer[12] << 8 | data_buffer[13]; // Z - YAW

	if (read_acc) {
		accel_raw[0] = -(data_buffer[2] << 8 | data_buffer[3]); //Y
		accel_raw[1] = (data_buffer[0] << 8 | data_buffer[1]); //X
		accel_raw[2] = data_buffer[4] << 8 | data_buffer[5]; //Z

		temp_raw = data_buffer[6] << 8 | data_buffer[7];
		temperature = (float) temp_raw / 340.f + 36.53f;
	}
	return HAL_OK;
}
HAL_StatusTypeDef Gyro::filtering(void) {

	for (int axis = 0; axis < 3; axis++) {
		float gyro_f;
		gyro_f = gyro_raw[axis] / gyro_scale;
		gyro_f = kalman[axis]->apply(gyro_f);
		gyro_f = notch1[axis]->apply(gyro_f);
		gyro_f = notch2[axis]->apply(gyro_f);
		rates[axis] = lpf2[axis]->apply(lpf1[axis]->apply(gyro_f));

	}
	if (read_acc) {
		AccUpdatedAtLeastOnce = true;
		for (int axis = 0; axis < 3; axis++) {
			float acc_f = accel_raw[axis] / acc_scale;
			g_rates[axis] = acc_lpf[axis]->apply(acc_f);

			AccAccumulatedMeasurements[axis] += g_rates[axis];
		}
		++AccAccumulatedMeasurementCount;
	}
	return HAL_OK;
}
HAL_StatusTypeDef Gyro::calfunction(void) {
	if (calibration_mode) {
		static uint32_t cal_inc = 0;
		for (int axis = 0; axis < 3; axis++) {
			cal_data[axis] += gyro_raw[axis];
			if (read_acc)
				acc_cal_data[axis] += accel_raw[axis];
			devPush(&cal_var[axis], gyro_raw[axis]);

			static uint16_t flashing = 0;
			if (++flashing > 800) {
				flashing = 0;
				HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
			}
			buzzer.EnableMode(Buzzer::IMU_CALIBRATION);
			gyro_raw[axis] = 0;
			angles[axis] = 0.f;
			GyroAccumulatedMeasurements[axis] = 0.f;
			GyroAccumulatedMeasurementTime = 0.f;

			accel_raw[axis] = 0;
			g_rates[axis] = 0.f;
			AccAccumulatedMeasurements[axis] = 0.f;
			AccAccumulatedMeasurementCount = 0;

			if (cal_inc > 100) {
				if (devStandardDeviation(&cal_var[axis]) > gyroMovementCalibrationThreshold) {
					for (int axis = 0; axis < 3; axis++) {
						cal_data[axis] = 0;
						acc_cal_data[axis] = 0;
						devClear(&cal_var[axis]);
					}
					cal_inc = 0;
					return HAL_BUSY;
				}
			}
		}
		if (++cal_inc >= CAL_CYCLES) {

			cal_inc = 0;
			for (int axis = 0; axis < 3; axis++) {
				const float stddev = devStandardDeviation(&cal_var[axis]);
				if (stddev > gyroMovementCalibrationThreshold) {
					for (int axis = 0; axis < 3; axis++) {
						cal_data[axis] = 0;
						acc_cal_data[axis] = 0;
						devClear(&cal_var[axis]);
					}
	                return HAL_BUSY;
	            }
				GYRO_ZRO[axis] = (int16_t)roundf((float)cal_data[axis] / (float)CAL_CYCLES);
				ACC_ZRO[axis] = (int16_t)roundf((float)acc_cal_data[axis] * 8.f / (float)CAL_CYCLES);
				if(axis == Z)
					ACC_ZRO[axis] -= (int16_t)acc_scale;

				acc_cal_data[axis] = 0;
				cal_data[axis] = 0;
			}
			calibration_mode = 0;
			flash_led_3times = 6;
			buzzer.EnableMode(Buzzer::THREE_BEEPS);
		}
	} else
		;
	return HAL_OK;
}
HAL_StatusTypeDef Gyro::zeroing(void) {
	for (int axis = 0; axis < 3; axis++)
		gyro_raw[axis] -= GYRO_ZRO[axis];
	if(read_acc){
		for (int axis = 0; axis < 3; axis++)
				accel_raw[axis] -= ACC_ZRO[axis];
	}
	return HAL_OK;
}
HAL_StatusTypeDef Gyro::accumulations(void) {
	static float gyro_previous[3] = {0.f, 0.f, 0.f};
	static float rise = 0.f;
	for (uint8_t axis = 0; axis < 3; axis++)
	{
		float delta = 0.5f * (rates[axis] + gyro_previous[axis]) * dT;
		if (axis == Z) {
			rise += delta;
			int16_t i = (int16_t) (rise * YAW_INVPRECISION);
			rise -= i * YAW_PRECISION;
			float yaw_tmp = angles[Z] -i * YAW_PRECISION; // "-" SIGN TO MATCH AHRS REFERENCE FRAME

			if (yaw_tmp < 0.f)
				angles[Z] = yaw_tmp + 360.f;
			else if (yaw_tmp >= 360.f)
				angles[Z] = yaw_tmp - 360.f;
			else
				angles[Z] = yaw_tmp;
		}

		GyroAccumulatedMeasurements[axis] += delta;
		gyro_previous[axis] = rates[axis];

	}
	GyroAccumulatedMeasurementTime += dT;
	return HAL_OK;
}
void Gyro::filters(void) {
	for (int axis = 0; axis < 3; axis++) {
		/**** KALMAN */
		if (kalman_qrpi[0] > 0.f && kalman_qrpi[1] > 0.f)
			kalman[axis] = new KalmanFilter(kalman_qrpi[0], kalman_qrpi[1],
					kalman_qrpi[2], kalman_qrpi[3]);
		else
			kalman[axis] = new NullFilter();
		/**** KALMAN */

		/**** NOTCH1 */
		if (gyro_notch1_hz[axis] > 0.f && gyro_notch1_cut[axis] > 0.f)
			notch1[axis] = new BiquadFilter(FILTER_NOTCH, dT, gyro_notch1_hz[axis], gyro_notch1_cut[axis]);
		else
			notch1[axis] = new NullFilter();
		/**** NOTCH1 */

		/**** NOTCH2 */
		if (gyro_notch2_hz[axis] > 0.f && gyro_notch2_cut[axis] > 0.f)
			notch2[axis] = new BiquadFilter(FILTER_NOTCH, dT,
					gyro_notch2_hz[axis], gyro_notch2_cut[axis]);
		else
			notch2[axis] = new NullFilter();
		/**** NOTCH2 */

		/**** LPF1 */
		if (gyro_lpf1_type == LPF_PT1 && gyro_lpf1_hz[axis] > 0.f)
			lpf1[axis] = new PT1Filter(gyro_lpf1_hz[axis], dT);
		else if (gyro_lpf1_type == LPF_BIQUAD && gyro_lpf1_hz[axis] > 0.f)
			lpf1[axis] = new BiquadFilter(FILTER_LPF, dT, gyro_lpf1_hz[axis]);
		else
			lpf1[axis] = new NullFilter();
		/**** LPF1 */

		/**** LPF2 */
		if (gyro_lpf2_type == LPF_PT1 && gyro_lpf2_hz[axis] > 0.f)
			lpf2[axis] = new PT1Filter(gyro_lpf2_hz[axis], dT);
		else if (gyro_lpf2_type == LPF_BIQUAD && gyro_lpf2_hz[axis] > 0.f)
			lpf2[axis] = new BiquadFilter(FILTER_LPF, dT, gyro_lpf2_hz[axis]);
		else
			lpf2[axis] = new NullFilter();
		/**** LPF2 */

		/**** ACC */
		if (acc_lpf_type == LPF_PT1 && acc_lpf_hz[axis] > 0.f)
			acc_lpf[axis] = new PT1Filter(acc_lpf_hz[axis], (float)acc_denom * dT);
		else if (acc_lpf_type == LPF_BIQUAD && acc_lpf_hz[axis] > 0.f)
			acc_lpf[axis] = new BiquadFilter(FILTER_LPF, (float)acc_denom * dT, acc_lpf_hz[axis]);
		else
			acc_lpf[axis] = new NullFilter();
		/**** ACC */
	}
}

void Gyro::StartCalibration(void) {
	if (!calibration_mode) {
		CAL_CYCLES = CALIBRATION_TIME/dT;
		for (int axis = 0; axis < 3; axis++) {
			GYRO_ZRO[axis] = 0;
			devClear(&cal_var[axis]);
		}
		ahrs.OrientationReset();
		calibration_mode = 1;
	}
}
bool Gyro::isAccelUpdatedAtLeastOnce(void){
	return AccUpdatedAtLeastOnce;
}
bool Gyro::GetGyroAccumulationAverage(float *accumulationAverage)
{
    if (GyroAccumulatedMeasurementTime > 0.f) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = GyroAccumulatedMeasurements[axis] / GyroAccumulatedMeasurementTime;
            GyroAccumulatedMeasurements[axis] = 0.0f;
        }
        GyroAccumulatedMeasurementTime = 0.f;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}
bool Gyro::GetAccAccumulationAverage(float *accumulationAverage)
{
    if (AccAccumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = AccAccumulatedMeasurements[axis] / AccAccumulatedMeasurementCount;
            AccAccumulatedMeasurements[axis] = 0.0f;
        }
        AccAccumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}
float Gyro::getGyroScale(void){
	return gyro_scale;
}
float Gyro::getAccScale(void){
	return acc_scale;
}
HAL_StatusTypeDef Gyro::mpu_init(void) {

	uint8_t status = HAL_OK;
	gyro_on = 0;

	HAL_SPI_DeInit(&hspi1);
	osDelay(1);
	status |= Gyro_SPI_Init(_SPI_CLOCK_INITIALIZATON);
	osDelay(1);

	// ICM20602 RESET
	status |= write(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	osDelay(150);
	status |= write(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	osDelay(150);


	// TOGGLE-OFF SLEEPMODE
	status |= write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	osDelay(1);

	// Disable Primary I2C Interface
	status |= write(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
	osDelay(1);
	status |= write(MPU_RA_PWR_MGMT_2, 0x00);
	osDelay(1);
	// GYRO CONFIG
	status |= write(MPU_RA_GYRO_CONFIG, INV_FSR_500DPS << 3);
	osDelay(1);

	// ACC CONFIG
	status |= write(MPU_RA_ACCEL_CONFIG, INV_FSR_4G << 3);
	osDelay(1);
//	// LPF CONFIG
//	status |= write(MPU_RA_CONFIG, BITS_DLPF_CFG_256HZ);
//	osDelay(1);
	// RATE CONFIG - FULL SAMPLING
	status |= write(MPU_RA_SMPLRT_DIV, 0 );
	osDelay(1);
	// DATA READY INTERRUPT
	status |= write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);
	osDelay(1);
	status |= write(MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
	osDelay(1);
	HAL_SPI_DeInit(&hspi1);
	osDelay(1);
	status |= Gyro_SPI_Init(_SPI_CLOCK_FAST);
	osDelay(1);
	gyro_on = 1;
	status = HAL_OK;
	return (HAL_StatusTypeDef) status;
}
void Gyro::DataReady(void) {
	if (gyro_on) {
		osSignalSet(GyroTaskHandle, Signal_DR);
	}
}
void Gyro::TxCpltCallback(void) {
	osSignalSet(GyroTaskHandle, Signal_TX);
}
void Gyro::RxCpltCallback(void) {
	osSignalSet(GyroTaskHandle, Signal_RX);

}
HAL_StatusTypeDef Gyro::read(uint8_t reg_addr, uint8_t *data, uint32_t size) {

	uint8_t status = HAL_OK;

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);

	status |= HAL_SPI_Transmit_IT(&hspi1, &reg_addr, 1);
	osSignalWait(Signal_TX, 1);
	status |= HAL_SPI_Receive_DMA(&hspi1, data, size);
	osSignalWait(Signal_RX, 1);
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
	return (HAL_StatusTypeDef) status;
}
HAL_StatusTypeDef Gyro::write(uint8_t gyro_reg, uint8_t data) {
	uint8_t status = HAL_OK;

	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET);

	status |= HAL_SPI_Transmit_IT(&hspi1, &gyro_reg, 1);
	osSignalWait(Signal_TX, 1);
	status |= HAL_SPI_Transmit_IT(&hspi1, &data, 1);
	osSignalWait(Signal_TX, 1);
	HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);

	return (HAL_StatusTypeDef) status;
}
Gyro::~Gyro() {
	for (int axis = 0; axis < 3; axis++) {
		delete kalman[axis];
		delete notch1[axis];
		delete notch2[axis];
		delete lpf1[axis];
		delete lpf2[axis];
	}
}

