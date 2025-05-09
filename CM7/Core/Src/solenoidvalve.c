/*
 * solenoidvalve.c
 *
 *  Created on: Apr 2, 2025
 *      Author: USER
 */

#include "solenoidvalve.h"
#include <stdint.h>

void sv_single_con(uint8_t* ch)
{
	for (uint8_t i = 0; i<MAX_SV_NUM; i++) {
		switch(i) {
		case 0: // SV CH1
			if (ch[0] == 0) {
				HAL_GPIO_WritePin(SV_CH1_PORT, SV_CH1_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH1_PORT, SV_CH1_GPIO, GPIO_PIN_SET);
			}
			break;
		case 1:
			if (ch[1] == 0) {
				HAL_GPIO_WritePin(SV_CH2_PORT, SV_CH2_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH2_PORT, SV_CH2_GPIO, GPIO_PIN_SET);
			}
			break;
		case 2:
			if (ch[2] == 0) {
				HAL_GPIO_WritePin(SV_CH3_PORT, SV_CH3_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH3_PORT, SV_CH3_GPIO, GPIO_PIN_SET);
			}
			break;
		case 3:
			if (ch[3] == 0) {
				HAL_GPIO_WritePin(SV_CH4_PORT, SV_CH4_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH4_PORT, SV_CH4_GPIO, GPIO_PIN_SET);
			}
			break;
		default:
			break;
		}
	}

}




