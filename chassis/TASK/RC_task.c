#include "rc_task.h"
#include "remote_control.h"
#include "bsp_math.h"
#include "gimbal_task.h"
#include "vision_task.h" 
#include "shoot_task.h"


int calibrate_start_flag=0;

static int deadline_judge(uint8_t a);
//死区判断
static int deadline_judge(uint8_t a) 
{

		if(abs(a-RC_MIDD)<=DEADLINE) return 1;
	    else return 0;
}

//遥控器控制模式
void remote_control_data(void)
{
	calibrate_start_flag=0;


		/*                模式选择                 */
	if(switch_is_up(rc_ctrl.rc.s[1]))  //左上  
	{
		One_Shoot_flag=0;
        Ten_Shoot_flag=0;
		vision_mode=VISION_ON;
		rc_shoot.left_fric.target_speed = SHOOT_FRIC_SPEED;
		rc_shoot.right_fric.target_speed = -SHOOT_FRIC_SPEED;

		if(switch_is_up(rc_ctrl.rc.s[0]) ) // 
		{
			gimbal_set_mode = VISION_ON_ALL; 
		}
		if(switch_is_mid(rc_ctrl.rc.s[0])) // 
		{
			gimbal_set_mode = VOLUNTARIY_PATROL;
		}
		if(switch_is_down(rc_ctrl.rc.s[0])) //  
		{
			//calibrate_start_flag=1;
			return;
		}
		
	}
	if(switch_is_mid(rc_ctrl.rc.s[1])) // 中 
	{
		vision_mode=VISION_OFF;
		if(switch_is_up(rc_ctrl.rc.s[0])) // 开
		{
//			gimbal_set_mode = VOLUNTARIY_PATROL;
			if(deadline_judge(rc_ctrl.rc.ch[1])==0)
				rc_sent.x_speed=limits_change(X_SPEED_MAXX,X_SPEED_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MINN);
			else rc_sent.x_speed=0;	
		}
		if(switch_is_mid(rc_ctrl.rc.s[0])) // 中
		{
			gimbal_set_mode =VISION_OFF_ALL;
			if(deadline_judge(rc_ctrl.rc.ch[1])==0)
				rc_sent.x_speed=limits_change(X_SPEED_MAXX,X_SPEED_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MINN);
			else rc_sent.x_speed=0;	
			if(deadline_judge(rc_ctrl.rc.ch[2])==0)
			{
				rc_sent.yaw.target_speed=limits_change(RC_YAW_SPEED_MAXX,RC_YAW_SPEED_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
				rc_sent.yaw.target_angle=-limits_change(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
			}
			else
			{
				rc_sent.yaw.target_speed=0;
				rc_sent.yaw.target_angle=0;
			}
			if(deadline_judge(rc_ctrl.rc.ch[3])==0)
			{
				rc_sent.pitch.target_speed=limits_change(RC_PITCH_SPEED_MAXX,RC_PITCH_SPEED_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
				rc_sent.pitch.target_angle=-limits_change(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
			}
			else
			{
				rc_sent.pitch.target_speed=0;
				rc_sent.pitch.target_angle=0;
			}

		}
		if(switch_is_down(rc_ctrl.rc.s[0]))
		{
			
		}
	}
	if(switch_is_down(rc_ctrl.rc.s[1])) // 下  发子弹
	{
		rc_shoot.left_fric.target_speed = SHOOT_FRIC_SPEED;
		rc_shoot.right_fric.target_speed = -SHOOT_FRIC_SPEED;
		if(switch_is_up(rc_ctrl.rc.s[0]))// 单发
		{
			One_Shoot_flag=1;
		}
		if(switch_is_mid(rc_ctrl.rc.s[0])) // 关
		{
				One_Shoot_flag=0;
                Ten_Shoot_flag=0;
		}
		if(switch_is_down(rc_ctrl.rc.s[0])) //  连发
		{
			Ten_Shoot_flag=1;
		}
	}
	
	
}
