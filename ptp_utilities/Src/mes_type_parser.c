/*
 * mes_type.c
 *
 *  Created on: May 31, 2024
 *      Author: muzix
 */
#include <mes_type_parser.h>
#include <stdint.h>

#define MESSAGE_TYPE					0

uint32_t swapEndian(uint32_t value) {
    uint32_t result = 0;

    // Extract and swap each byte of the 32-bit value
    result |= (value & 0x000000FF) << 24; // Move byte 0 to byte 3
    result |= (value & 0x0000FF00) << 8;  // Move byte 1 to byte 2
    result |= (value & 0x00FF0000) >> 8;  // Move byte 2 to byte 1
    result |= (value & 0xFF000000) >> 24; // Move byte 3 to byte 0

    return result;
}

uint8_t read_message_type(void **payload)
{
	uint8_t *byte_ptr = (uint8_t *)(*payload);
	return *byte_ptr;
};

void read_announce_message(MsgAnnounce *announce, void **payload)
{
	void *ptr = *payload;
	// announce->originTimestamp.secondsField.high = *(uint32_t *)((uint8_t*)ptr + 34);
	// announce->originTimestamp.secondsField.low = *(uint16_t *)((uint8_t*)ptr + 40);
	announce->currentUtcOffset = *(int16_t *)((uint8_t*)ptr + 44);
	announce->grandmasterPriority1 = *(uint8_t *)((uint8_t*)ptr + 47);
	announce->grandmasterClockQuality.clockClass = *(uint8_t *)((uint8_t *)ptr + 48);
	announce->grandmasterClockQuality.clockAccuracy = *(uint8_t *)((uint8_t *)ptr + 49);
	announce->grandmasterClockQuality.offsetScaledLogVariance = *(uint16_t *)((uint8_t *)ptr + 50);
	announce->grandmasterPriority2 = *(uint8_t *)((uint8_t *)ptr + 52);
	for(int i = 0; i < 8; i ++)
	{
		announce->grandmasterIdentity.identity[i] = *(uint8_t *)((uint8_t *)ptr + 53 + i);
	}
	announce->stepsRemoved = *(int16_t *)((uint8_t *)ptr + 61);
	announce->timeSource = *(uint8_t *)((uint8_t *)ptr + 63);
};

void read_sync_message(MsgSync *sync, void *payload)
{
	sync->originTimestamp.secondsField = swapEndian(*(uint32_t *)((uint8_t*)payload + 36));
	sync->originTimestamp.nanosecondsField = swapEndian(*(uint32_t *)((uint8_t*)payload + 40));
}

