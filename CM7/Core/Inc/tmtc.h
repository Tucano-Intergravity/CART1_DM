/*
 * tmtc.h
 *
 *  Created on: May 27, 2025
 *      Author: USER
 */

#ifndef INC_TMTC_H_
#define INC_TMTC_H_

#include <stdint.h>

#define MAX_TC_SIZE 128

extern uint8_t fTC;

void InitTMTC(void);
void GetTC(uint8_t*);
void SendTM(uint8_t*);

#endif /* INC_TMTC_H_ */
