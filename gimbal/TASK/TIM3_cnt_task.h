#ifndef __TIM3_CNT_task_H
#define __TIM3_CNT_task_H

#include "tim.h"

typedef enum{
    VISION_ON= 0,  //×ÔÃé¿ª
    VISION_OFF,    //
}VISION_t;

#define TIM3_CNT_TASK()    HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

#endif
