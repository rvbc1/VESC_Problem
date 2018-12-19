/*
 * Futaba.h
 *
 *  Created on: 23.04.2018
 *      Author: mice
 */

#ifndef CLASSES_FUTABA_H_
#define CLASSES_FUTABA_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "sbus.h"
#include "cmsis_os.h"

#include "Filters.h"

class Futaba {
	uint8_t RxBuffer[8];
	sbusFrame_t frame;

	uint8_t RCState = 3;
	uint8_t frameStatus = 0;
	uint32_t startAtUs = 0;
	uint16_t stateFlags = 0;
	uint8_t position = 0;
	bool done = false;

	uint16_t midrc = 1500;

	Filter* lpf[4];
	void sbusChannelsInit(void);
	void sbusDataReceive(uint8_t c);
	uint8_t sbusChannelsDecode(void);
	uint8_t sbusFrameStatus(void);
	uint16_t sbusChannelsReadRawRC(uint8_t chan);
	void Conversions(void);
	void RCCommands(void);

	void ApplyOffsets(void);
	void ArmingRoutine(void);
public:
	uint32_t frames_dropped = 0;
	int32_t RxSignal = 1 << 0;
	uint16_t sbusChannelData[SBUS_MAX_CHANNEL];

	float StickDeflection[4];
	float SmoothDeflection[4];
	/* A B C potek potek D E F */
	ThreePositionSwitch_e SwitchA = SWITCH_UP;
	ThreePositionSwitch_e SwitchB = SWITCH_UP;
	ThreePositionSwitch_e SwitchC = SWITCH_UP;

	ThreePositionSwitch_e SwitchD = SWITCH_UP;
	ThreePositionSwitch_e SwitchE = SWITCH_UP;
	ThreePositionSwitch_e SwitchF = SWITCH_UP;

	bool Stick_Command[SBUS_MAX_CHANNEL];

	void Init(void);
	void ConfigureSmoothing(float cutoff, float _dt);
	void Process(void);
	void ProcessSmoothing(void);
	uint16_t Get_RCState(void);
	void RxCallback(void);

	Futaba();
	virtual ~Futaba();
};
extern Futaba futaba;

typedef enum {
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
    RX_FRAME_DROPPED = (1 << 3)
} rxFrameState_e;
#ifdef __cplusplus
 }
#endif
#endif /* CLASSES_FUTABA_H_ */
