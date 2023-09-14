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
uint8_t top_flag=0; //С���ݱ�־λ 0 û�� 1 ��
uint8_t auto_exposure_flag=0;  //�Զ��ع��־λ
uint8_t reboot_flag=0;   //�������������־λ
uint8_t traget_exit_flag=1;   //����Ȩ��־λ 0 ����Ȩ������λ��  1 ����Ȩ������λ��
uint8_t flag_upper=0;  //�����õ�
int8_t upper_computer_data[8];


void receive_upper_data(void)
{
				//yaw
	 vision_sent.yaw.target_angle =	(float)((upper_computer_data[1]<<8|(upper_computer_data[0]&0xff))/100.0f);
		    
				//pitch		
	 vision_sent.pitch.target_angle = (float)((upper_computer_data[3]<<8|(upper_computer_data[2]&0xff))/100.0f);
				
	//����Ȩ	������Ҫ�޸�:�����
	if(target_exit_rec_flag)
	{
		traget_exit_flag = upper_computer_data[4];
		target_exit_rec_flag = 0;		
	}
	
}
