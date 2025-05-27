/*
 * pressure.h
 *
 *  Created on: May 27, 2025
 *      Author: USER
 */

#ifndef INC_PRESSURE_H_
#define INC_PRESSURE_H_

#include <stdint.h>

#define N_ADC_CH 13

void InitPT(void);

void GetADCRaw(uint16_t*);

void ME750_GetPressure(double*);

#endif /* INC_PRESSURE_H_ */
