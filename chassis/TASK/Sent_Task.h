#ifndef __sent_task_H
#define __sent_task_H

#include "stm32f4xx.h"

uint8_t canTX_chassis_first(int16_t x,int16_t y,int16_t z,int16_t  current_t);
uint8_t canTX_chassis_second(int16_t mode);
uint8_t canTX_gimbal(int16_t pitch,int16_t yaw);
uint8_t canTX_fric(int16_t left,int16_t right,int16_t trigger);
uint8_t canTX_trigger(int16_t trigger);
#endif
