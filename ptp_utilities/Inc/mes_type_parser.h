/*
 * mes_type.h
 *
 *  Created on: May 31, 2024
 *      Author: muzix
 */
#include <stdint.h>

#ifndef INC_MES_TYPE_PARSER_H_
#define INC_MES_TYPE_PARSER_H_


#define SYNC 					0x00
#define DELAY_REQ 				0x01
#define PDELAY_REQ				0x02
#define PDELAY_RESP 			0x03

#define FOLLOW_UP				0x08
#define DELAY_RESP				0x09
#define PDELAY_RESP_FOLLOW_UP 	0x0A
#define ANNOUNCE				0x0B
#define SIGNALING				0x0C
#define MANAGEMENT				0x0D

/*Set data structure for int48 for nanoseconds*/
typedef struct {
	uint32_t high;
	uint16_t low;
} int48_bit;

/**
 * \brief data structure of clock identity
 */
typedef struct {
    uint8_t identity[8];
} ClockIdentity;

/**
* \brief 5.3.7 The ClockQuality represents the quality of a clock
 */

typedef struct
{
		uint8_t clockClass;
		uint8_t clockAccuracy;
		uint16_t offsetScaledLogVariance;
} ClockQuality;

/**
 * \brief The Timestamp type represents a positive time with respect to the epoch
 */

typedef struct
{
		int48_bit secondsField;
		uint32_t nanosecondsField;
} Timestamp_message;

/**
 * \brief Announce message fields (Table 25 of the spec)
 */

typedef struct
{
		Timestamp_message originTimestamp;
		int16_t currentUtcOffset;
		uint8_t grandmasterPriority1;
		ClockQuality grandmasterClockQuality;
		uint8_t grandmasterPriority2;
		ClockIdentity grandmasterIdentity;
		int16_t stepsRemoved;
		uint8_t timeSource;
}MsgAnnounce;

uint8_t read_message_type(void **payload);

void read_announce_message(MsgAnnounce *announce, void **payload);

#endif /* INC_MES_TYPE_PARSER_H_ */
