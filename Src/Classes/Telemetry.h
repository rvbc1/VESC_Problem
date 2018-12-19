/*
 * Telemetry.h
 *
 *  Created on: 23.06.2018
 *      Author: mice
 */

#ifndef CLASSES_TELEMETRY_H_
#define CLASSES_TELEMETRY_H_

#include "stdint.h"

#define MAX_DATAIDS 17
typedef struct smartPortPayload_s {
	uint8_t frameId;
	uint16_t valueId;
	uint32_t data;
}__attribute__((packed)) smartPortPayload_t;

class Telemetry {
	int32_t Signal_TX = 1 << 0;
	int32_t Signal_RX = 1 << 1;
	uint8_t RxBuffer[128];
	uint8_t serialRxBytesWaiting = 0;
	uint8_t TxBuffer[32];

	bool telemetry_on = false;

	typedef struct frSkyTableInfo_s {
		uint16_t * table;
		uint8_t size;
		uint8_t index;
	} frSkyTableInfo_t;
	frSkyTableInfo_t frSkyDataIdTableInfo = { frSkyDataIdTable, 0, 0 };

	uint16_t frSkyDataIdTable[MAX_DATAIDS];


	typedef struct smartPortFrame_s {
		uint8_t sensorId;
		smartPortPayload_t payload;
		uint8_t crc;
	}__attribute__((packed)) smartPortFrame_t;

	void initSensors(void);
	uint8_t serialRead(void);
	void handleTelemetry(void);
	void processTelemetry(smartPortPayload_t *payload, volatile bool *clearToSend, const uint32_t *requestTimeout);
	smartPortPayload_t * smartPortDataReceive(uint16_t c, bool *clearToSend, bool useChecksum);

	void SendPackage(uint16_t id, uint32_t val);
	void WriteFrame(const smartPortPayload_t *payload, uint16_t checksum);
public:
	void Init(void);
	void Process(void);
	void TxCallback(void);
	void RxCallback(void);
	Telemetry();
	virtual ~Telemetry();
};
extern Telemetry telemetry;

#endif /* CLASSES_TELEMETRY_H_ */
