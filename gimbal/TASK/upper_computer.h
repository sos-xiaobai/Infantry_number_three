#ifndef __UPPER_COMPUTER_H
#define __UPPER_COMPUTER_H

#include "stdint.h"

extern int8_t upper_computer_data[8];
extern uint8_t flag_upper;
extern uint8_t windwill_flag;
extern uint8_t top_flag; //小陀螺标志位 0 没开 1 开

extern uint8_t reboot_flag;   //自瞄程序重启标志位
extern uint8_t traget_exit_flag;   //控制权标志位 0 控制权交给下位机  1 控制权交给上位机

void receive_upper_data(void);

#endif
