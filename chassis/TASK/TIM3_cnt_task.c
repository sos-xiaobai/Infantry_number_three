#include "tim3_cnt_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "tim.h"
#include "bsp_imu.h"
#include "rc_task.h"
#include "calibrate_task.h"
#include "bsp_uart.h"

int time_count=0;
int IMU_cnt=0,start_flag=0,shoot_flag;
//0.1ms
void TIM3_CNT_TASK()
{
     if(IMU_cnt>10) start_flag=1;
	 time_count++;

		INS_task();
     if(time_count%13==0&&start_flag==1)
	 {
		//校准imu
		if(calibrate_start_flag==1)
		calibrate_task();
		//云台任务
		Gimbal_Task();
		//发射任务
	  shoot_task();
		//发送给上位机
		DMA_Send();
	 }
		  
	 if(time_count%7==0)
	 {  	    
		 //底盘发送
		remote_chassis();
		//遥控器控制
		remote_control_data();
		 //发送给上位机
		DMA_Send();
	 }
	 
			
	 if(time_count>=1000) 
	 {
		 time_count=0;
		 if(start_flag==0)
		 IMU_cnt++;
	 }

}
