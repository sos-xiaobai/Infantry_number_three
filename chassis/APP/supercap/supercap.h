#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "usart.h"

//extern uint16_t supercap_volt;
//extern uint16_t supercap_per;
//void supercap(void);
//void get_supercap_data(void);

#define Low_Voltage_Mode 		0
#define Medium_Voltage_Mode		1
#define High_Voltage_Mode 		2

#define	Supercap_Connected		1
#define Supercap_Disconnected	0
/***********************************************************
*@Brief	供外部调用的变量
***********************************************************/
extern float supercap_volt;
extern float supercap_per;
extern int Supercap_Connection_Status;
/***********************************************************
*@Brief	供外部调用的函数
***********************************************************/
void supercap(int S_Cnt, int MS_Cnt);
#endif

