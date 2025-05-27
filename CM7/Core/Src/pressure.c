/*
 * pressure.c
 *
 *  Created on: May 27, 2025
 *      Author: USER
 */

#include <string.h>
#include "pressure.h"
#include "stm32h7xx_hal.h"

uint16_t adc_buffer[N_ADC_CH];
extern ADC_HandleTypeDef hadc1;

extern void Error_Handler(void);


void InitPT(void)
{
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, N_ADC_CH) != HAL_OK)
    {
        Error_Handler();
    }
}

void GetADCRaw(uint16_t* buf)
{
	memcpy((void*)buf, (void*)adc_buffer, sizeof(uint16_t)*N_ADC_CH);
}

void ME750_GetPressure(double* pPres)
{

}

//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}



