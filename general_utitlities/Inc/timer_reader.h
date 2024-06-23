/*
 * timer_reader.h
 *
 *  Created on: Jun 9, 2024
 *      Author: muzix
 */

#ifndef TIMER_READER_H_
#define TIMER_READER_H_

#include "stm32f7xx_hal_eth.h"

void getTime_Wrapper(ETH_HandleTypeDef *heth, ETH_TimeTypeDef *time)
{
	if(HAL_ETH_PTP_GetTime(heth, time) != HAL_OK)
	{
		// Print Error!
	}
}

#endif /* TIMER_READER_H_ */
