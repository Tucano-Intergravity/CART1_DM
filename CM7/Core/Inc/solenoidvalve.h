/*
 * solenoidvalve.h
 *
 *  Created on: Apr 2, 2025
 *      Author: USER
 */

#ifndef INC_SOLENOIDVALVE_H_
#define INC_SOLENOIDVALVE_H_

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

#define MAX_SV_NUM 10

#define SV_CH_1		1001
#define SV_CH_2		1002
#define SV_CH_3		1003
#define SV_CH_4		1004
#define SV_CH_5		1005
#define SV_CH_6		1006
#define SV_CH_7		1007
#define SV_CH_8		1008
#define SV_CH_9		1009
#define SV_CH_10	1010

#define SV_CH1_PORT		GPIOE
#define SV_CH1_GPIO		GPIO_PIN_0

#define SV_CH2_PORT		GPIOE
#define SV_CH2_GPIO		GPIO_PIN_3

#define SV_CH3_PORT		GPIOE
#define SV_CH3_GPIO		GPIO_PIN_4

#define SV_CH4_PORT		GPIOE
#define SV_CH4_GPIO		GPIO_PIN_7

#define SV_CH5_PORT		GPIOE
#define SV_CH5_GPIO		GPIO_PIN_8

#define SV_CH6_PORT		GPIOE
#define SV_CH6_GPIO		GPIO_PIN_11

#define SV_CH7_PORT		GPIOE
#define SV_CH7_GPIO		GPIO_PIN_12

#define SV_CH8_PORT		GPIOE
#define SV_CH8_GPIO		GPIO_PIN_15

#define SV_CH9_PORT		GPIOC
#define SV_CH9_GPIO		GPIO_PIN_6

#define SV_CH10_PORT	GPIOC
#define SV_CH10_GPIO	GPIO_PIN_9

void sv_single_con(uint8_t* ch);

#endif /* INC_SOLENOIDVALVE_H_ */
