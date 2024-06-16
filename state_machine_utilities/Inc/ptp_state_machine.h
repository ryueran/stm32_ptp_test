/*
 * ptp_state_machine.h
 *
 *  Created on: Jun 8, 2024
 *      Author: muzix
 */

#ifndef INC_PTP_STATE_MACHINE_H_
#define INC_PTP_STATE_MACHINE_H_

typedef enum PTP_SLAVE_STATE {INIT, ANNOUNCED, SYNCED, FOLLOWED_UP} ptp_slave_state;

#define SYNC_TRIGGERED			0x00
#define FOLLOW_UP_TRIGGERED		0x08
#define ANNOUNCE_TRIGGERED		0x0B

void state_transition(ptp_slave_state state, uint8_t event)
{
	switch(state) {
	case INIT:
		if(event == ANNOUNCE_TRIGGERED)
		{
			state = ANNOUNCED;
		} else {
			state = INIT;
		}
	case ANNOUNCED:
		if(event == SYNC_TRIGGERED)
		{
			state = SYNCED;
		} else {
			state = ANNOUNCED;
		}
	case SYNCED:
		if(event == ANNOUNCE_TRIGGERED)
		{
			state = ANNOUNCED;
		} else if(event == FOLLOW_UP_TRIGGERED)
		{
			state = FOLLOWED_UP;
		} else {
			state = SYNCED;
		}
	}
}

struct ptp_state_info {
	ptp_slave_state state;
	void (*state_trans)(ptp_slave_state state, uint8_t event);
} ptp_information;

#endif /* INC_PTP_STATE_MACHINE_H_ */
