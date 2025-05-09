/*
 * thermocouple.h
 *
 *  Created on: Mar 19, 2025
 *      Author: USER
 */

#ifndef INC_THERMOCOUPLE_H_
#define INC_THERMOCOUPLE_H_

#define U1_CS_PORT 	GPIOD
#define U1_CS_PIN	GPIO_PIN_0

#define U2_CS_PORT 	GPIOD
#define U2_CS_PIN	GPIO_PIN_1

#define U3_CS_PORT 	GPIOD
#define U3_CS_PIN	GPIO_PIN_2

#define U4_CS_PORT 	GPIOD
#define U4_CS_PIN	GPIO_PIN_3

#define U5_CS_PORT 	GPIOD
#define U5_CS_PIN	GPIO_PIN_4

#define U6_CS_PORT 	GPIOD
#define U6_CS_PIN	GPIO_PIN_5

#define U7_CS_PORT 	GPIOD
#define U7_CS_PIN	GPIO_PIN_6

#define U8_CS_PORT 	GPIOD
#define U8_CS_PIN	GPIO_PIN_7

#define U9_CS_PORT 	GPIOD
#define U9_CS_PIN	GPIO_PIN_11

#define U10_CS_PORT 	GPIOD
#define U10_CS_PIN	GPIO_PIN_12

#define U11_CS_PORT 	GPIOD
#define U11_CS_PIN	GPIO_PIN_13

#define U12_CS_PORT 	GPIOD
#define U12_CS_PIN	GPIO_PIN_14


#define MAX_TC_CH	20

#define TC_CH1	0
#define TC_CH2 	1
#define TC_CH3 	2
#define TC_CH4 	3
#define TC_CH5 	4
#define TC_CH6 	5
#define TC_CH7 	6
#define TC_CH8 	7
#define TC_CH9 	8
#define TC_CH10	9
#define TC_CH11	10
#define TC_CH12	11

/*
 * thermocouple.c
 *
 *  Created on: Mar 19, 2025
 *      Author: USER
 */
#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "thermocouple.h"

#define SPI_TIMEOUT 100  // SPI 통신 타임아웃
extern SPI_HandleTypeDef hspi1; // SPI1 핸들러 외부 선언

// CS 핀을 Low로 설정 (SPI 통신 시작)
void MAX31855_CS_Enable(uint8_t ch);

// CS 핀을 High로 설정 (SPI 통신 종료)
void MAX31855_CS_Disable(uint8_t ch);

// SPI를 통해 MAX31855에서 32비트 데이터 읽기
uint32_t MAX31855_ReadData(uint8_t ch);

// 열전대(Hot Junction) 온도 변환 함수
float MAX31855_GetThermocoupleTemperature(uint8_t ch);

// 내부(Cold Junction) 온도 변환 함수
//float MAX31855_GetInternalTemperature(void);

// 최종 온도 계산 함수
//float MAX31855_GetFinalTemperature(uint8_t ch);

// 오류 상태 확인 함수
void MAX31855_CheckFault(uint8_t);



#endif /* INC_THERMOCOUPLE_H_ */
