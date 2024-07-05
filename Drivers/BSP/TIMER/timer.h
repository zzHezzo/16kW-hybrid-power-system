#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"

void TIM9_PWM_Init(uint16_t arr,uint16_t psc,uint16_t pulse);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void TIM_SetTIM9Compare2(uint32_t compare);
#endif

