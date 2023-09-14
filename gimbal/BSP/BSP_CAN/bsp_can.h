#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx.h"
#include "can.h"

uint8_t Bsp_canInit(void);
void CAN_cmd_chassis_reset_ID(void);

#endif
