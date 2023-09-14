#ifndef __MY_anonymous_H
#define __MY_anonymous_H
#include "usart.h"



/*------------------------------------------------------------------------------------------------------
【函    数】蓝牙传输数据
【功    能】       
【参    数】
【返 回 值】
【实    例】
【注意事项】  
--------------------------------------------------------------------------------------------------------*/
void Data_send_to_ANO(void);
void Data_Send_F1(int *pst, unsigned char len);
void Bluetooth_transmission(void);



#endif
