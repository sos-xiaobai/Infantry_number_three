#include "chassis_task.h"
#include "supercap.h"
#include "MY_anonymous.h"
#include "referee_UI.h"
/**
  * @breif         底盘所有任务函数，实质上是定时器中断，1ms进入一次
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
int IMU_cnt=0,start_flag=0,calibrate_start_flag;
int MS_Count=0;
int S_Count=0;
void Time_Service_Task(void);


void CHASSIS_TASK()
{
	static int time_count=0;

	if(IMU_cnt>10) start_flag=1;
	 time_count++;
	Time_Service_Task();
		INS_task();
	if(time_count%13==0&&start_flag==1)
	 {
		//校准imu
		if(calibrate_start_flag==1)
			calibrate_task();
		IMU_read();
	 }
	
	//chassis_control_order.chassis_mode=0;
	if(time_count%10==0)
	{
		Get_Base_Velocities();
		send_gimbal_data_2();
		canTX_chassis_imu();
		//读取裁判系统
		referee_unpack_fifo_data();
		//向云台发送数据
		send_gimbal_data();
	}
	
	
	if(time_count%7==0)
	{
		//远程遥控，调试时使用
//		remote_control();
		//底盘运动
		chassis_move();
		canTX_gimbal_y(chassis_center.target_current); //直接发送云台传过来的电流赋值
		
	}
	
//	if(time_count%5==0)
//	{
//		
//	}
	
	if(time_count>=1000)			//清除计数标志    1s
	{
		time_count=1;
    //超级电容
	
		if(start_flag==0)
		 IMU_cnt++;
	}
	supercap(S_Count, MS_Count);

}
void Time_Service_Task(void)
{
	MS_Count++;
	if(MS_Count>999)			
	{
		MS_Count=0;
		S_Count++;
	}
}

