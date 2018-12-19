/*
 * Telemetry.cpp
 *
 *  Created on: 23.06.2018
 *      Author: mice
 */

#include <Telemetry.h>
#include "smartport.h"

#include "Allshit.h"
#include "PowerManager.h"
#include "Mathematics.h"
#include "AHRS.h"
#include "Gyro.h"
#include "Motor.h"
#include "Tools.h"

#include "usart.h"

#define ADD_SENSOR(dataId) frSkyDataIdTableInfo.table[frSkyDataIdTableInfo.index++] = dataId
#define SMARTPORT_MSP_PAYLOAD_SIZE (sizeof(smartPortPayload_t) - sizeof(uint8_t))
#define SMARTPORT_SERVICE_TIMEOUT_MS 1 // max allowed time to find a value to send

Telemetry telemetry;


void Telemetry::Init(void){
	MX_UART4_Init();
	initSensors();
	telemetry_on = true;
}
void Telemetry::Process(void) {
	HAL_HalfDuplex_EnableReceiver(&huart4);
	uint8_t tmp;
	HAL_UART_Receive_IT(&huart4, (uint8_t*)&tmp, 1);
	osEvent evt = osSignalWait(Signal_RX, 1);
	if (evt.status == osEventSignal) {
		RxBuffer[serialRxBytesWaiting] = tmp;
		serialRxBytesWaiting++;
	} else if (serialRxBytesWaiting && evt.status == osEventTimeout) {
		handleTelemetry();
	}
}
void Telemetry::initSensors(void)
{
    frSkyDataIdTableInfo.index = 0;

    ADD_SENSOR(FSSP_DATAID_T1);
	ADD_SENSOR(FSSP_DATAID_T2);


	ADD_SENSOR(FSSP_DATAID_VFAS);

	ADD_SENSOR(FSSP_DATAID_A4);

	// CURRENT SENSOR
	if (0) {
		ADD_SENSOR(FSSP_DATAID_CURRENT);
		ADD_SENSOR(FSSP_DATAID_FUEL);
	}

	ADD_SENSOR(FSSP_DATAID_HEADING);
	ADD_SENSOR(FSSP_DATAID_ACCX);
	ADD_SENSOR(FSSP_DATAID_ACCY);
	ADD_SENSOR(FSSP_DATAID_ACCZ);

	ADD_SENSOR(FSSP_DATAID_ALTITUDE);
	ADD_SENSOR(FSSP_DATAID_VARIO);
	ADD_SENSOR(FSSP_DATAID_RPM);
	ADD_SENSOR(FSSP_DATAID_SPEED);
	ADD_SENSOR(FSSP_DATAID_TEMP);

	ADD_SENSOR(FSSP_DATAID_GPS_ALT);

    frSkyDataIdTableInfo.size = frSkyDataIdTableInfo.index;
    frSkyDataIdTableInfo.index = 0;
}
smartPortPayload_t * Telemetry::smartPortDataReceive(uint16_t c, bool *clearToSend, bool useChecksum)
{
    static uint8_t rxBuffer[sizeof(smartPortPayload_t)];
    static uint8_t smartPortRxBytes = 0;
    static bool skipUntilStart = true;
    static bool awaitingSensorId = false;
    static bool byteStuffing = false;
    static uint16_t checksum = 0;

    if (c == FSSP_START_STOP) {
        *clearToSend = false;
        smartPortRxBytes = 0;
        awaitingSensorId = true;
        skipUntilStart = false;
        return NULL;
    } else if (skipUntilStart) {
        return NULL;
    }

    if (awaitingSensorId) {

        awaitingSensorId = false;
        if ((c == FSSP_SENSOR_ID1) && serialRxBytesWaiting == 0) {
            // our slot is starting, no need to decode more
            *clearToSend = true;
            skipUntilStart = true;
        } else if (c == FSSP_SENSOR_ID2) {
            checksum = 0;
        } else {
            skipUntilStart = true;
        }
    } else {
        if (c == FSSP_DLE) {
            byteStuffing = true;

            return NULL;
        } else if (byteStuffing) {
            c ^= FSSP_DLE_XOR;
            byteStuffing = false;
        }

        if (smartPortRxBytes < sizeof(smartPortPayload_t)) {
            rxBuffer[smartPortRxBytes++] = (uint8_t)c;
            checksum += c;
            if (!useChecksum && (smartPortRxBytes == sizeof(smartPortPayload_t))) {
                skipUntilStart = true;

                return (smartPortPayload_t *)&rxBuffer;
            }
        } else {
            skipUntilStart = true;

            checksum += c;
            checksum = (checksum & 0xFF) + (checksum >> 8);
            if (checksum == 0xFF) {
                return (smartPortPayload_t *)&rxBuffer;
            }
        }
    }

    return NULL;
}
void Telemetry::handleTelemetry(void)
{
    static bool clearToSend = false;
    static smartPortPayload_t *payload = NULL;

    const uint32_t requestTimeout = HAL_GetTick() + SMARTPORT_SERVICE_TIMEOUT_MS;

	if (telemetry_on) {
		while (serialRxBytesWaiting > 0 && !payload) {
			uint8_t c = serialRead();
			payload = smartPortDataReceive(c, &clearToSend, true);
		}

		processTelemetry(payload, &clearToSend, &requestTimeout);
		payload = NULL;
	}
}
void Telemetry::processTelemetry(smartPortPayload_t *payload, volatile bool *clearToSend, const uint32_t *requestTimeout)
{
    static uint8_t smartPortIdCycleCnt = 0;
    static uint8_t t1Cnt = 0;
    static uint8_t t2Cnt = 0;
    UNUSED(t2Cnt);

    bool doRun = true;
    while (doRun && *clearToSend) {
        // Ensure we won't get stuck in the loop if there happens to be nothing available to send in a timely manner - dump the slot if we loop in there for too long.
        if (requestTimeout) {
            if (HAL_GetTick()>= *requestTimeout) {
                *clearToSend = false;

                return;
            }
        } else {
            doRun = false;
        }

        // we can send back any data we want, our tables keep track of the order and frequency of each data type we send
        frSkyTableInfo_t * tableInfo = &frSkyDataIdTableInfo;

            if (tableInfo->index == tableInfo->size) { // end of table reached, loop back
                tableInfo->index = 0;
            }

        uint16_t id = tableInfo->table[tableInfo->index];

        smartPortIdCycleCnt++;
        tableInfo->index++;

		int32_t tmpi;
		uint32_t tmp1 = 0;
		uint16_t vfasVoltage;
		uint8_t cellCount;
		uint32_t rpm, temp;

		switch (id) {
		case FSSP_DATAID_VFAS:
			vfasVoltage = lrintf(powermanager.voltage * 100.f);

			SendPackage(id, vfasVoltage);
			*clearToSend = false;
			break;

		case FSSP_DATAID_CURRENT:
			SendPackage(id, lrintf(powermanager.amperage * 10.f)); // given in 10mA steps, unknown requested unit
			*clearToSend = false;
			break;

		case FSSP_DATAID_ALTITUDE:
			SendPackage(id, 0); // unknown given unit, requested 100 = 1 meter
			*clearToSend = false;
			break;
		case FSSP_DATAID_FUEL:
			SendPackage(id, lrintf(powermanager.consumption)); // given in mAh, unknown requested unit
			*clearToSend = false;
			break;
		case FSSP_DATAID_VARIO:
			SendPackage(id, lrintf(motor.getVelocity()/ 10.f));
			*clearToSend = false;
			break;
		case FSSP_DATAID_HEADING:
			SendPackage(id, lrintf(ahrs.attitude.values.yaw * 100.f)); // given in 10*deg, requested in 10000 = 100 deg
			*clearToSend = false;
			break;
		case FSSP_DATAID_ACCX:
			SendPackage(id, lrintf(100 * gyro.g_rates[X])); // Multiply by 100 to show as x.xx g on Taranis
			*clearToSend = false;
			break;
		case FSSP_DATAID_ACCY:
			SendPackage(id, lrintf(100 * gyro.g_rates[Y]));
			*clearToSend = false;
			break;
		case FSSP_DATAID_ACCZ:
			SendPackage(id, lrintf(100 * gyro.g_rates[Z]));
			*clearToSend = false;
			break;
		case FSSP_DATAID_T1:
			tmp1 = lrintf(gyro.temperature);
			SendPackage(id, (uint32_t) tmp1);
			*clearToSend = false;
			break;
		case FSSP_DATAID_T2:
			temp = lrintf(powermanager.temperature);
			SendPackage(id, temp);
			*clearToSend = false;
			break;
		case FSSP_DATAID_A4:
			cellCount = powermanager.GetBatteryCells();
			vfasVoltage = cellCount ? ((powermanager.voltage * 100.f) / (float) cellCount) : 0; // given in 0.1V, convert to volts
			SendPackage(id, vfasVoltage);
			*clearToSend = false;
			break;
		case FSSP_DATAID_RPM:
			rpm = lrintf(motor.getRPMs());
			SendPackage(id, rpm);
			*clearToSend = false;
			break;
		case FSSP_DATAID_SPEED:
			SendPackage(id, lrintf(motor.getVelocity() * 1943.8445f));
			*clearToSend = false;
			break;
		case FSSP_DATAID_TEMP:
			// we send all the flags as decimal digits for easy reading

			// the t1Cnt simply allows the telemetry view to show at least some changes
			t2Cnt++;
			if (t2Cnt == 4) {
				t2Cnt = 1;
			}
			tmpi = t2Cnt * 10000; // start off with at least one digit so the most significant 0 won't be cut off
			// the Taranis seems to be able to fit 5 digits on the screen
			// the Taranis seems to consider this number a signed 16 bit integer

			//TODO - Wysylanie flag samochodu
			SendPackage(id, (uint32_t) tmpi);
			*clearToSend = false;
			break;
		case FSSP_DATAID_GPS_ALT:
			SendPackage(id, 0);
			*clearToSend = false;
			break;
		default:
			break;
			// if nothing is sent, hasRequest isn't cleared, we already incremented the counter, just loop back to the start
		}
	}
}
uint8_t Telemetry::serialRead(void)
{
	uint8_t byte = RxBuffer[0];
	serialRxBytesWaiting--;
	for (uint8_t index = 0; index < serialRxBytesWaiting; index++) {
		RxBuffer[index] = RxBuffer[index + 1];
	}
	return byte;
}
void Telemetry::SendPackage(uint16_t id, uint32_t val)
{
    smartPortPayload_t payload;
    payload.frameId = FSSP_DATA_FRAME;
    payload.valueId = id;
    payload.data = val;

    WriteFrame(&payload, 0);
}
void Telemetry::WriteFrame(const smartPortPayload_t *payload, uint16_t checksum)
{
	TxBuffer[0] = payload->frameId;
	TxBuffer[1] = payload->valueId & 0xff;
	TxBuffer[2] = (payload->valueId >> 8) & 0xff;
	TxBuffer[3] = payload->data & 0xff;
	TxBuffer[4] = (payload->data >> 8) & 0xff;
	TxBuffer[5] = (payload->data >> 16) & 0xff;
	TxBuffer[6] = (payload->data >> 24) & 0xff;

	for (uint8_t i = 0; i < 7; i++) {
		checksum += TxBuffer[i];
	}
	checksum = 0xff - ((checksum & 0xff) + (checksum >> 8));
	TxBuffer[7] = checksum & 0xff;

    HAL_HalfDuplex_EnableTransmitter(&huart4);
	HAL_UART_Transmit_DMA(&huart4, TxBuffer, 8);
	osSignalWait(Signal_TX,2);

}
void Telemetry::TxCallback(void) {
	osSignalSet(TelemetryTaskHandle, Signal_TX);
}
void Telemetry::RxCallback(void){
	osSignalSet(TelemetryTaskHandle, Signal_RX);
}
Telemetry::Telemetry() {
	// TODO Auto-generated constructor stub

}
Telemetry::~Telemetry() {
	// TODO Auto-generated destructor stub
}

