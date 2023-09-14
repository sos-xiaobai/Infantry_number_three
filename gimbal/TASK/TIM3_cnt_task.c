#include "tim3_cnt_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "tim.h"
#include "bsp_imu.h"
#include "rc_task.h"
#include "calibrate_task.h"
#include "bsp_uart.h"
#include "sent_task.h"
#include "upper_computer.h"

extern VISION_t vision_mode;
extern float vision_yaw,vision_pitch; //定点位置
int time_count=0,count_2=0,time_count_2=0;
int IMU_cnt=0,start_flag=0;
//0.1ms

void prevent_jam(void);
void shake_three(void);
void long_pre_shoot(void);
//任务汇总
void TIM3_CNT_TASK()
{
	if(htim->Instance==TIM3)
  {
	  time_count++;
//		INS_task();
		upper_computer_reboot();  //按键软件重启上位机
   if(time_count%13==0&&start_flag==1)
	 {
		 Gimbal_Task();
		 prevent_jam();
		 shoot_task();
		  
		 canTX_UPPER_COMPUTER();  //向上位机发送数据
		 canTX_UPPER_COMPUTER_2();
		 receive_upper_data();   //处理上位机发来的数据
	 } 
	 if(time_count%7==0)
	 {  	    
		remote_chassis();  //选择模式
		control_mode_judge();  //判断控制来源  遥控器/键鼠
		if(control_mode==KEY_OFF)
		 remote_control_data();
		else  //键鼠模式
		{
			 key_control_data();
			judge_key();				
		}
//    long_pre_shoot();	
	 }
	 if(time_count>=1000) 
		 time_count=0;
  }
  if(htim->Instance==TIM4)  //定时器4处理imu任务
	{
		count_2++;
	 if(IMU_cnt>10) start_flag=1; 
		INS_task();
   if(count_2>=1000) 
	 {
		 count_2=0;
		 if(start_flag==0)
		 IMU_cnt++;
	 }	
	}
}
int shake_flag=0,cnt_,One_shoot_trigger_time_count=0,Five_shoot_trigger_time_count=0; 
uint8_t shoot_jam_flag=0;
int trigger_actual_angle;
int yuyuyuyyuyu=5500;

void prevent_jam(void) 
{
			if((abs(rc_shoot.trigger.actual_current)>yuyuyuyyuyu)&&(shoot_jam_flag==0))  //未达到目标角度 18429
			{
				One_shoot_trigger_time_count++;
        if(One_shoot_trigger_time_count>=80) 
				{
					trigger_actual_angle=rc_shoot.trigger.target_angle;
		       rc_shoot.trigger.target_angle=rc_shoot.trigger.total_angle+18429;
				   One_shoot_trigger_time_count=0;
					
					shoot_jam_flag=1;
					Five_shoot_trigger_time_count=0;
				}
			}
			else One_shoot_trigger_time_count=0;
			
			if(shoot_jam_flag==1)
			{
				Five_shoot_trigger_time_count++;
				if(Five_shoot_trigger_time_count>=150)
				{
					rc_shoot.trigger.target_angle=trigger_actual_angle;
					Five_shoot_trigger_time_count=0;
					shoot_jam_flag=0;
				}
			}
}

void shake_three(void)  //抖动三下
{
	shake_flag=1;
	if(shake_flag==1)
	{
		cnt_++;
		if(cnt_>=1000)
		{
			rc_shoot.trigger.target_angle=rc_shoot.trigger.total_angle-5/360*36859;
			shake_flag=2;
			cnt_=0;
		}
	}
	if(shake_flag==2)
	{
		cnt_++;
		if(cnt_>=1000)
		{
			rc_shoot.trigger.target_angle=rc_shoot.trigger.total_angle+5/360*36859;
			shake_flag=3;
		}
	}
	if(shake_flag==3)
	{
		cnt_++;
		if(cnt_>=1000)
		{
			rc_shoot.trigger.target_angle=rc_shoot.trigger.total_angle+5/360*36859;
			shake_flag=4;
		}
	}
}

uint16_t shoot_count=0;
void long_pre_shoot(void)
{
	if(MOUSE_pre_right==1)
		shoot_count++;
	else shoot_count=0;
	if(shoot_count>=200)
	{
		if(abs(rc_shoot.trigger.target_angle-rc_shoot.trigger.total_angle)<500)  //视为这一发子弹已经拨出
			rc_shoot.trigger.target_angle-=((1.0f*36.0f*(360.0f/8.0f))/360.0f*8191.0f);
	}
}