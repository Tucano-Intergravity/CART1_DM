/*
 * tmtc.c
 *
 *  Created on: May 27, 2025
 *      Author: USER
 */

#include "tmtc.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <string.h>

extern UART_HandleTypeDef huart4;

uint8_t fTC = false;

uint8_t rx_data;
uint8_t rx_buffer[MAX_TC_SIZE];
uint8_t rx_index = 0;
uint8_t tc_buffer[MAX_TC_SIZE];
uint8_t NRecv = 0;

extern void Error_Handler(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		rx_buffer[rx_index++] = rx_data;

		if (rx_data == 0x0A)
		{
			fTC = true;
			memcpy((void*)tc_buffer, (void*)rx_buffer, sizeof(uint8_t)*rx_index);
			NRecv = rx_index;
			rx_index = 0;
		}

		// 순환 버퍼 또는 수신 종료 문자 조건으로 처리 가능
		if (rx_index >= MAX_TC_SIZE)
		{
			rx_index = 0; // 오버플로 방지
		}

		// 다음 수신 재시작
		HAL_UART_Receive_IT(&huart4, &rx_data, 1);
	}
}

void InitTMTC(void)
{
	if (HAL_UART_Receive_IT(&huart4, &rx_data, 1) != HAL_OK)
	{
		Error_Handler();
	}
}

void GetTC(uint8_t* TC)
{
	memcpy((void*)TC,(void*)tc_buffer,sizeof(uint8_t)*NRecv);
}

void SendTM(uint8_t* TM)
{
	uint8_t n_send = strlen((char*)TM);
	HAL_UART_Transmit(&huart4, (uint8_t*)TM, n_send, HAL_MAX_DELAY);
}
