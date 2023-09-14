#ifndef __MY_INIT_H
#define __MY_INIT_H

#include "pid.h"
#include "chassis_move.h"
#include "tim.h"
#include "remote_control.h"
#include "bsp_referee.h"
#include "bsp_imu.h"
#include "lqr.h"
#include "BMI088driver.h"
#include "calibrate_task.h"
#include "delay.h"
#include "led.h"

void all_init(void);
void mode_init(void);
#endif

