#ifndef __TIM3_CNT_task_H
#define __TIM3_CNT_task_H

#include "tim.h"

#define TIM3_CNT_TASK()    HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

#endif
