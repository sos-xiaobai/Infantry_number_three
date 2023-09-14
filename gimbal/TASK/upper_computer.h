#ifndef __UPPER_COMPUTER_H
#define __UPPER_COMPUTER_H

#include "stdint.h"

extern int8_t upper_computer_data[8];
extern uint8_t flag_upper;
extern uint8_t windwill_flag;
extern uint8_t top_flag; //С���ݱ�־λ 0 û�� 1 ��

extern uint8_t reboot_flag;   //�������������־λ
extern uint8_t traget_exit_flag;   //����Ȩ��־λ 0 ����Ȩ������λ��  1 ����Ȩ������λ��

void receive_upper_data(void);

#endif
