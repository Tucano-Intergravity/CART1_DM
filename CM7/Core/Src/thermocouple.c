/*
 * thermocouple.c
 *
 *  Created on: Mar 19, 2025
 *      Author: USER
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "thermocouple.h"

extern SPI_HandleTypeDef hspi1; // SPI1 핸들러 외부 선언

// CS 핀을 Low로 설정 (SPI 통신 시작)
void MAX31855_CS_Enable(uint8_t ch)
{
	switch(ch) {
	case TC_CH1:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH2:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH3:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH4:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH5:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH6:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH7:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH8:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH9:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH10:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH11:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH12:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}

// CS 핀을 High로 설정 (SPI 통신 종료)
void MAX31855_CS_Disable(uint8_t ch)
{
	switch(ch) {
	case TC_CH1:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH2:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH3:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH4:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH5:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH6:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH7:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH8:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH9:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH10:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH11:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	case TC_CH12:
		HAL_GPIO_WritePin(U1_CS_PORT, U1_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U2_CS_PORT, U2_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U3_CS_PORT, U3_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U4_CS_PORT, U4_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U5_CS_PORT, U5_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U6_CS_PORT, U6_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U7_CS_PORT, U7_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U8_CS_PORT, U8_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U9_CS_PORT, U9_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U10_CS_PORT, U10_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U11_CS_PORT, U11_CS_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(U12_CS_PORT, U12_CS_PIN, GPIO_PIN_SET);
		break;
	default:
		break;
	}
}

// SPI를 통해 MAX31855에서 32비트 데이터 읽기
uint32_t MAX31855_ReadData(uint8_t ch)
{
	uint8_t rxBuffer[4] = {0}; // 4바이트(32비트) 데이터 버퍼
	uint32_t rawData = 0;

	MAX31855_CS_Enable(ch); // CS Low → SPI 시작
	HAL_SPI_Receive(&hspi1, rxBuffer, 4, HAL_MAX_DELAY); // 32비트(4바이트) 데이터 수신
	MAX31855_CS_Disable(ch); // CS High → SPI 종료

	// 4바이트 데이터를 32비트 정수로 변환
	rawData = ((uint32_t)rxBuffer[0] << 24) |
			((uint32_t)rxBuffer[1] << 16) |
			((uint32_t)rxBuffer[2] << 8) |
			((uint32_t)rxBuffer[3]);

	if (rawData & 0x00010000) {
		printf("Fault detected!\n");
	}
	if (rawData & 0x00000004) {
		printf("SCV Fault: Thermocouple shorted to VCC!\n");
	}
	if (rawData & 0x00000002) {
		printf("SCG Fault: Thermocouple shorted to GND!\n");
	}
	if (rawData & 0x00000001) {
		printf("OC Fault: Thermocouple open circuit!\n");
	}

	return rawData;
}

// 열전대(Hot Junction) 온도 변환 함수
float MAX31855_GetThermocoupleTemperature(uint8_t ch)
{
	uint32_t rawData = MAX31855_ReadData(ch);
	int16_t tempData = (rawData >> 18) & 0x3FFF; // 상위 14비트 추출

	// 음수 보정 (2의 보수 변환)
	if (rawData & 0x80000000)
	{
		tempData |= 0xC000; // 부호 확장
	}

	return (float)tempData * 0.25; // 해상도 0.25°C 변환
}

// 내부(Cold Junction) 온도 변환 함수
//float MAX31855_GetInternalTemperature(void)
//{
//	uint32_t rawData = MAX31855_ReadData();
//	int16_t coldJunctionData = (rawData >> 4) & 0x0FFF; // 상위 12비트 추출
//
//	// 음수 보정 (2의 보수 변환)
//	if (coldJunctionData & 0x800)
//	{
//		coldJunctionData |= 0xF000; // 부호 확장
//	}
//
//	return (float)coldJunctionData * 0.0625; // 해상도 0.0625°C 변환
//}

// 최종 온도 계산 함수
//float MAX31855_GetFinalTemperature(void)
//{
//    float thermocoupleTemp = MAX31855_GetThermocoupleTemperature(); // 열전대 온도 읽기
//    float coldJunctionTemp = MAX31855_GetInternalTemperature(); // 내부 기준 온도 읽기
//
//    return thermocoupleTemp; // + coldJunctionTemp; // 최종 온도 계산
//}

// 오류 상태 확인 함수
void MAX31855_CheckFault(uint8_t ch)
{
	uint32_t rawData = MAX31855_ReadData(ch);

	if (rawData & 0x00010000) {
		printf("Fault detected!\n");
	}
	if (rawData & 0x00000004) {
		printf("SCV Fault: Thermocouple shorted to VCC!\n");
	}
	if (rawData & 0x00000002) {
		printf("SCG Fault: Thermocouple shorted to GND!\n");
	}
	if (rawData & 0x00000001) {
		printf("OC Fault: Thermocouple open circuit!\n");
	}
}


//void GetTemp(double* Temp)
//{
//	for (uint8_t i=0; i < MAX_TC_CH; i++)
//	{
//		Temp[i] = (double)MAX31855_GetThermocoupleTemperature(i);
//	}
//}

uint8_t fTemp = false;
uint8_t idx_tc = 0;
uint8_t dummy_tx[4] = {0xFF, 0xFF, 0xFF, 0xFF};
uint8_t spi_rx_buffer[4];
double tc[MAX_TC_CH] = {0};

void MAX31855_ReadTemp_IT(uint8_t ch)
{
	MAX31855_CS_Enable(idx_tc);
    //HAL_SPI_TransmitReceive_IT(&hspi1, dummy_tx, spi_rx_buffer, 4);
    HAL_SPI_Receive_IT(&hspi1, spi_rx_buffer, 4);

}

void MAX3188_StartRead()
{
	idx_tc = 0;
	fTemp = false;

	MAX31855_ReadTemp_IT(idx_tc);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
    	MAX31855_CS_Disable(idx_tc);

    	uint8_t data[4];
    	memcpy((void*)data,(void*)spi_rx_buffer,sizeof(uint8_t)*4);

        int32_t value = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) |
                        ((int32_t)data[2] << 8)  | ((int32_t)data[3]);

        int16_t temp_data = (value >> 18) & 0x3FFF; // 14비트 Thermocouple data는 bit[31:18]에 위치

        if (temp_data & 0x2000)
        {
            temp_data |= 0xC000; // Sign 확장
        }

        tc[idx_tc] = (double)temp_data * 0.25f;

        idx_tc++;

		if (idx_tc == MAX_TC_CH)
		{
			idx_tc = 0;
			fTemp = true;
		}
		else
		{
			MAX31855_ReadTemp_IT(idx_tc);
		}
    }
}



void GetTemp(double* Temp)
{
	for (uint8_t i = 0; i < MAX_TC_CH; i++)
	{
		Temp[i] = tc[i];
	}
}


//double Convert_Temp(uint8_t *data) {
//    int32_t value = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) |
//                    ((int32_t)data[2] << 8)  | ((int32_t)data[3]);
//
//    // 14비트 Thermocouple data는 bit[31:18]에 위치
//    int16_t temp_data = (value >> 18) & 0x3FFF;
//
//    if (temp_data & 0x2000) {
//        temp_data |= 0xC000; // Sign 확장
//    }
//
//    return (double)temp_data * 0.25f;
//}


