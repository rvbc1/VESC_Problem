/*
 * Futaba.cpp
 *
 *  Created on: 23.04.2018
 *      Author: mice
 */

#include "Futaba.h"

#include "Allshit.h"

#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"

#include "Tools.h"
#include "Buzzer.h"

#define SBUS_TIME_NEEDED_PER_FRAME 3000

#define RX_OFFSET_T 192
#define RX_OFFSET_AER 992

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)
#define SBUS_FRAME_BEGIN_BYTE 0x0F

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

Futaba futaba;
void Futaba::sbusDataReceive(uint8_t c){

	const uint32_t nowUs = tools.GetMicros();
	const int32_t sbusFrameTime = nowUs - startAtUs;

	if (sbusFrameTime > (long) (SBUS_TIME_NEEDED_PER_FRAME + 500)) {
		position = 0;
	}

	if (position == 0) {
		if (c != SBUS_FRAME_BEGIN_BYTE) {
			return;
		}
		startAtUs = nowUs;
	}

	if (position < SBUS_FRAME_SIZE) {
		frame.bytes[position++] = (uint8_t) c;
		if (position < SBUS_FRAME_SIZE) {
			done = false;
		} else {
			done = true;
		}
	}
}
uint8_t Futaba::sbusFrameStatus(void){

	    if (!done) {
	        return RX_FRAME_PENDING;
	    }
	    done = false;

	    if (frame.frame.channels.flags & SBUS_FLAG_SIGNAL_LOSS) {
	        stateFlags |= SBUS_STATE_SIGNALLOSS;
	    }
	    if (frame.frame.channels.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
	        stateFlags |= SBUS_STATE_FAILSAFE;
	    }

	    return sbusChannelsDecode();
}
uint8_t Futaba::sbusChannelsDecode(void) {
	sbusChannelData[0] = frame.frame.channels.chan0;
	sbusChannelData[1] = frame.frame.channels.chan1;
	sbusChannelData[2] = frame.frame.channels.chan2;
	sbusChannelData[3] = frame.frame.channels.chan3;
	sbusChannelData[4] = frame.frame.channels.chan4;
	sbusChannelData[5] = frame.frame.channels.chan5;
	sbusChannelData[6] = frame.frame.channels.chan6;
	sbusChannelData[7] = frame.frame.channels.chan7;
	sbusChannelData[8] = frame.frame.channels.chan8;
	sbusChannelData[9] = frame.frame.channels.chan9;
	sbusChannelData[10] = frame.frame.channels.chan10;
	sbusChannelData[11] = frame.frame.channels.chan11;
	sbusChannelData[12] = frame.frame.channels.chan12;
	sbusChannelData[13] = frame.frame.channels.chan13;
	sbusChannelData[14] = frame.frame.channels.chan14;
	sbusChannelData[15] = frame.frame.channels.chan15;

	if (frame.frame.channels.flags & SBUS_FLAG_CHANNEL_17) {
		sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MAX;
	} else {
		sbusChannelData[16] = SBUS_DIGITAL_CHANNEL_MIN;
	}

	if (frame.frame.channels.flags & SBUS_FLAG_CHANNEL_18) {
		sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MAX;
	} else {
		sbusChannelData[17] = SBUS_DIGITAL_CHANNEL_MIN;
	}

	if (frame.frame.channels.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
		// internal failsafe enabled and rx failsafe flag set
		// RX *should* still be sending valid channel data (repeated), so use it.
		return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
	}

	if (frame.frame.channels.flags & SBUS_FLAG_SIGNAL_LOSS) {
		// The received data is a repeat of the last valid data so can be considered complete.
		return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
	}

	return RX_FRAME_COMPLETE;
}
uint16_t Futaba::sbusChannelsReadRawRC(uint8_t chan)
{
    // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
    // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
    return (5 * sbusChannelData[chan] / 8) + 880;
}

void Futaba::sbusChannelsInit(void)
{
    for (int b = 0; b < SBUS_MAX_CHANNEL; b++) {
    	sbusChannelData[b] = (16 * midrc) / 10 - 1408;
    	Stick_Command[b] = false;
    }
	StickDeflection[ROLL] = 0.f;
	StickDeflection[PITCH] = 0.f;
	StickDeflection[THROTTLE] = 0.f;
	StickDeflection[YAW] = 0.f;

	SwitchA = SWITCH_UP;
	SwitchB = SWITCH_UP;
	SwitchC = SWITCH_UP;

	SwitchD = SWITCH_UP;
	SwitchE = SWITCH_UP;
	SwitchF = SWITCH_UP;
}
void Futaba::Init(void) {
	MX_USART3_UART_Init();
	tools.Init();
	sbusChannelsInit();
	HAL_UART_Receive_IT(&huart3, RxBuffer, 1);
}
uint16_t Futaba::Get_RCState(void) {
	return RCState;
}
void Futaba::Process(void) {
	static bool even_once = false;
	osEvent evt = osSignalWait(RxSignal, 1000);
	if (evt.status == osEventSignal) {
		even_once = true;
		sbusDataReceive(RxBuffer[0]);
		frameStatus = sbusFrameStatus();
		if (frameStatus & RX_FRAME_COMPLETE) {
			if (frameStatus & RX_FRAME_FAILSAFE) {
				RCState = 1;
				sbusChannelsInit();
			} else if (frameStatus & RX_FRAME_DROPPED) {
				frames_dropped++;
//				RCState = 2;
//				sbusChannelsInit();
			} else {
				Conversions();
				RCCommands();
				RCState = 0;
			}

		}
		HAL_UART_Receive_IT(&huart3, RxBuffer, 1);
	}
	else if(even_once){
		// FAILSAFE!
		RCState = 3;
		sbusChannelsInit();
		HAL_UART_DeInit(&huart3);
		Init();

	}
}
void Futaba::ConfigureSmoothing(float cutoff, float _dt) {
	for (uint8_t i = 0; i < 4; i++)
		lpf[i] = new BiquadFilter(FILTER_LPF, _dt, cutoff);
}
void Futaba::ProcessSmoothing(void) {
	for (int i = 0; i < 4; i++)
		SmoothDeflection[i] = lpf[i]->apply(StickDeflection[i]);
}
void Futaba::Conversions(void) {
	StickDeflection[ROLL] = (sbusChannelData[0] - RX_OFFSET_AER) / 800.f;
	StickDeflection[PITCH] = (sbusChannelData[1] - RX_OFFSET_AER) / 800.f;
	StickDeflection[THROTTLE] = (sbusChannelData[2] - RX_OFFSET_T) / 1600.f;
	StickDeflection[YAW] = (sbusChannelData[3] - RX_OFFSET_AER) / 800.f;

	if (sbusChannelData[AUX1] < 500) {
		SwitchA = SWITCH_UP;
	} else if (sbusChannelData[AUX1] > 1500) {
		SwitchA = SWITCH_DOWN;
	} else {
		SwitchA = SWITCH_MIDDLE;
	}
	if (sbusChannelData[AUX2] < 500) {
		SwitchB = SWITCH_UP;
	} else if (sbusChannelData[AUX2] > 1500) {
		SwitchB = SWITCH_DOWN;
	} else {
		SwitchB = SWITCH_MIDDLE;
	}

	if (sbusChannelData[AUX3] < 500) {
		SwitchC = SWITCH_UP;
	} else if (sbusChannelData[AUX3] > 1500) {
		SwitchC = SWITCH_DOWN;
	} else {
		SwitchC = SWITCH_MIDDLE;
	}

	if (sbusChannelData[AUX5] < 500) {
		SwitchD = SWITCH_UP;
	} else if (sbusChannelData[AUX5] > 1500) {
		SwitchD = SWITCH_DOWN;
	} else {
		SwitchD = SWITCH_MIDDLE;
	}

	if (sbusChannelData[AUX4] < 500) {
		SwitchE = SWITCH_UP;
	} else if (sbusChannelData[AUX4] > 1500) {
		SwitchE = SWITCH_DOWN;
	} else {
		SwitchE = SWITCH_MIDDLE;
	}

	if (sbusChannelData[AUX6] < 500) {
		SwitchF = SWITCH_UP;
	} else if (sbusChannelData[AUX6] > 1500) {
		SwitchF = SWITCH_MIDDLE;
	} else {
		SwitchF = SWITCH_DOWN;
	}
}
void Futaba::RCCommands(void)
{
	/* STICK COMMAND 0 (   .)    (   .) */
	static uint16_t counter0 = 0;
	if (StickDeflection[THROTTLE] < 0.05f && StickDeflection[YAW] > 0.95f && StickDeflection[PITCH] < -0.95f && StickDeflection[ROLL] > 0.95f) {
		if (++counter0 == 111) {
			Stick_Command[0] = Stick_Command[0] ? false : true;
			buzzer.EnableMode(Buzzer::ONE_BEEP);
		}
	} else
		counter0 = 0;

	/* STICK COMMAND 1 (.   )    (.   ) */
	static uint16_t counter1 = 0;
	if (StickDeflection[THROTTLE] < 0.05f && StickDeflection[YAW] < -0.95f && StickDeflection[PITCH] < -0.95f && StickDeflection[ROLL] < -0.95f) {
		if (++counter1 == 111) {
			Stick_Command[1] = Stick_Command[1] ? false : true;
			buzzer.EnableMode(Buzzer::ONE_BEEP);
		}
	} else
		counter1 = 0;

	/* STICK COMMAND 2 ('   )    ('   ) */
	static uint16_t counter2 = 0;
	if (StickDeflection[THROTTLE] > 0.95f && StickDeflection[YAW] < -0.95f && StickDeflection[PITCH] > 0.95f && StickDeflection[ROLL] < -0.95f) {
		if (++counter2 == 111) {
			Stick_Command[2] = Stick_Command[2] ? false : true;
			buzzer.EnableMode(Buzzer::ONE_BEEP);
		}
	} else
		counter2 = 0;

	/* STICK COMMAND 3 (   ')    (   ') */
	static uint16_t counter3 = 0;
	if (StickDeflection[THROTTLE] > 0.95f && StickDeflection[YAW] > 0.95f && StickDeflection[PITCH] > 0.95f && StickDeflection[ROLL] > 0.95f) {
		if (++counter3 == 111) {
			Stick_Command[3] = Stick_Command[3] ? false : true;
			buzzer.EnableMode(Buzzer::ONE_BEEP);
		}
	} else
		counter3 = 0;

	/* STICK COMMAND 4 (.   )    (   .) */
	static uint16_t counter4 = 0;
	if (StickDeflection[THROTTLE] < 0.05f && StickDeflection[YAW] < -0.95f && StickDeflection[PITCH] < -0.95f && StickDeflection[ROLL] > 0.95f) {
		if (++counter4 == 111) {
			Stick_Command[4] = Stick_Command[4] ? false : true;
			buzzer.EnableMode(Buzzer::ONE_BEEP);
		}
	} else
		counter3 = 0;
}
void Futaba::RxCallback(void) {
	osSignalSet(FutabaTaskHandle, RxSignal);
}

Futaba::Futaba() {
	// TODO Auto-generated constructor stub

}

Futaba::~Futaba() {
	// TODO Auto-generated destructor stub
}

