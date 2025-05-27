/*
 * solenoidvalve.c
 *
 *  Created on: Apr 2, 2025
 *      Author: USER
 */

#include "solenoidvalve.h"
#include <stdint.h>

void SVUpdate(uint8_t* ch)
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
		case 4:
			if (ch[4] == 0) {
				HAL_GPIO_WritePin(SV_CH5_PORT, SV_CH5_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH5_PORT, SV_CH5_GPIO, GPIO_PIN_SET);
			}
			break;
		case 5:
			if (ch[5] == 0) {
				HAL_GPIO_WritePin(SV_CH6_PORT, SV_CH6_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH6_PORT, SV_CH6_GPIO, GPIO_PIN_SET);
			}
			break;
		case 6:
			if (ch[6] == 0) {
				HAL_GPIO_WritePin(SV_CH7_PORT, SV_CH7_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH7_PORT, SV_CH7_GPIO, GPIO_PIN_SET);
			}
			break;
		case 7:
			if (ch[7] == 0) {
				HAL_GPIO_WritePin(SV_CH8_PORT, SV_CH8_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH8_PORT, SV_CH8_GPIO, GPIO_PIN_SET);
			}
			break;
		case 8:
			if (ch[8] == 0) {
				HAL_GPIO_WritePin(SV_CH9_PORT, SV_CH9_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH9_PORT, SV_CH9_GPIO, GPIO_PIN_SET);
			}
			break;
		case 9:
			if (ch[9] == 0) {
				HAL_GPIO_WritePin(SV_CH10_PORT, SV_CH10_GPIO, GPIO_PIN_RESET);
			}
			else {
				HAL_GPIO_WritePin(SV_CH10_PORT, SV_CH10_GPIO, GPIO_PIN_SET);
			}
			break;
		default:
			break;
		}
	}

}




