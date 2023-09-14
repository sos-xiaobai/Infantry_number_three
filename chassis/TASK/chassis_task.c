#include "chassis_task.h"
#include "sent_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "bsp_uart.h"

uint8_t chassis_speed_x,chassis_speed_y;

uint16_t move_count=0;
short int f_move_flag=0;
short int l_move_flag=0;

int yuuuu=0;
void remote_chassis(void)
{

	if(gimbal_set_mode==VISION_ON_ALL) //全自动
	{
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.given_current);
		canTX_chassis_second(1);
	}	
	if(gimbal_set_mode==VOLUNTARIY_PATROL) //自动循环
	{
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.given_current);
		canTX_chassis_second(2);
	}
	if(gimbal_set_mode==VISION_OFF_ALL) //调试
	{
		canTX_chassis_first(rc_sent.x_speed,rc_sent.y_speed,rc_sent.r_speed,gimbal_y.given_current);
		canTX_chassis_second(3);
	}
}
