/*
 * Gyro.h
 *
 *  Created on: 02.01.2018
 *      Author: mice
 */

#ifndef MULTIROTOR_GYRO_H_
#define MULTIROTOR_GYRO_H_


#include "stdint.h"
#include "mpu6000.h"
#include "Filters.h"

class Gyro {
	//SPI HANDLING
	const uint32_t _SPI_CLOCK_FAST = SPI_BAUDRATEPRESCALER_8;
	const uint32_t _SPI_CLOCK_INITIALIZATON = SPI_BAUDRATEPRESCALER_256;

	uint8_t SPI1txBuffer[128];
	uint8_t SPI1rxBuffer[128];

	bool read_acc = true;
	uint8_t acc_denom = 8;

	uint32_t timeouts = 0;
	uint8_t flash_led_3times = 0;
	//

	float dT = 0.000125f;
	uint8_t data_buffer[14];
	int16_t gyro_raw[3];
	int16_t accel_raw[3];
	int16_t temp_raw;

	const float gyro_scale = 65.534f; // 500dps
	const float acc_scale = 8192.f;	// 4G

	float GyroAccumulatedMeasurements[3];
	float GyroAccumulatedMeasurementTime = 0.f;

	bool AccUpdatedAtLeastOnce = false;
	float AccAccumulatedMeasurements[3];
	uint32_t AccAccumulatedMeasurementCount = 0;


	uint8_t data_adress = MPU_RA_ACCEL_XOUT_H | 0x80;

	uint8_t gyro_on = 0;
	Filter* kalman[3];
	Filter* notch1[3];
	Filter* notch2[3];
	Filter* lpf1[3];
	Filter* lpf2[3];
	Filter* acc_lpf[3];

	/**** Filter settings */
	const bool GYRO_32khz = false;
	const float kalman_qrpi[4] = {0, 88, 0, 0}; // 600

	const float gyro_notch1_hz[3] = {0, 0, 0};
	const float gyro_notch1_cut[3] = {100, 100, 100};

	const float gyro_notch2_hz[3] = {0, 0, 0};
	const float gyro_notch2_cut[3] = {0, 0, 0};

	const LPFFilterType_e gyro_lpf1_type = LPF_PT1;
	const float gyro_lpf1_hz[3] = {90, 90, 90};

	const LPFFilterType_e gyro_lpf2_type = LPF_PT1;
	const float gyro_lpf2_hz[3] = {180, 180, 180};

	const LPFFilterType_e acc_lpf_type = LPF_BIQUAD;
	const float acc_lpf_hz[3] = { 10, 10, 10 };
	/**** Filter settings */

	uint32_t filtering_time;

	float CALIBRATION_TIME = 3.f;
	uint32_t CAL_CYCLES = 8000;
	int32_t cal_data[3] = { 0, 0, 0 };
	int32_t acc_cal_data[3] = { 0, 0, 0 };
	stdev_t cal_var[3];
	uint16_t gyroMovementCalibrationThreshold = 24;
	int16_t GYRO_ZRO[3] = { 0, 0, 0 };
	int16_t ACC_ZRO[3] = { 0, 0, 0 };

	int32_t Signal_DR = 1 << 0;
	int32_t Signal_TX = 1 << 1;
	int32_t Signal_RX = 1 << 2;

	HAL_StatusTypeDef mpu_init(void);
	void filters(void);

	HAL_StatusTypeDef data_read(void);
	HAL_StatusTypeDef data_conv(void);
	HAL_StatusTypeDef filtering(void);
	HAL_StatusTypeDef calfunction(void);
	HAL_StatusTypeDef zeroing(void);
	HAL_StatusTypeDef accumulations(void);

	HAL_StatusTypeDef read(uint8_t reg_addr, uint8_t *data, uint32_t size);
	HAL_StatusTypeDef write(uint8_t gyro_reg, uint8_t data);

public:
	struct stats_s {
		int16_t basetime = 0;
		int16_t spi_time = 0;
		int16_t conv_time = 0;
		int16_t filtering_time = 0;
		int16_t total_time = 0;
	} stats;
	uint8_t calibration_mode = 0;
	float g_rates[3] = {0, 0, 0};
	float temperature = 0;
	float rates[3] = {0, 0, 0};
	float angles[3] = {0, 0, 0};

	void StartCalibration(void);

	void Init(void);
	void Process(void);
	void DataReady(void);
	void TxCpltCallback(void);
	void RxCpltCallback(void);

	bool isAccelUpdatedAtLeastOnce(void);

	bool GetGyroAccumulationAverage(float *accumulationAverage);
	bool GetAccAccumulationAverage(float *accumulationAverage);

	float getGyroScale(void);
	float getAccScale(void);

	Gyro();
	virtual ~Gyro();


};

extern Gyro gyro;

#endif /* MULTIROTOR_GYRO_H_ */
