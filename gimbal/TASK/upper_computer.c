#include "upper_computer.h"
#include "main.h"
#include "stm32f4xx.h"
#include "bsp_uart.h"
#include "gimbal_task.h"
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "usart.h"
#include "vision_task.h"
#include "shoot_task.h"
#include "can_receive.h"

uint8_t windwill_flag=0;
uint8_t top_flag=0; //小陀螺标志位 0 没开 1 开
uint8_t auto_exposure_flag=0;  //自动曝光标志位
uint8_t reboot_flag=0;   //自瞄程序重启标志位
uint8_t traget_exit_flag=1;   //控制权标志位 0 控制权交给下位机  1 控制权交给上位机
uint8_t flag_upper=0;  //测试用的
int8_t upper_computer_data[8];


void receive_upper_data(void)
{
				//yaw
	 vision_sent.yaw.target_angle =	(float)((upper_computer_data[1]<<8|(upper_computer_data[0]&0xff))/100.0f);
		    
				//pitch		
	 vision_sent.pitch.target_angle = (float)((upper_computer_data[3]<<8|(upper_computer_data[2]&0xff))/100.0f);
				
	//控制权	后期需要修改:金旭峰
	if(target_exit_rec_flag)
	{
		traget_exit_flag = upper_computer_data[4];
		target_exit_rec_flag = 0;		
	}
	
}
