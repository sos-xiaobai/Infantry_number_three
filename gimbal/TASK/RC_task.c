#include "rc_task.h"
#include "remote_control.h"
#include "bsp_math.h"
#include "gimbal_task.h"
#include "vision_task.h" 
#include "bsp_uart.h"
#include "shoot_task.h"
#include "gimbal_task_behaviour.h"
#include "upper_computer.h"
#include "tim.h"

KEY_CONTROL control_mode=KEY_OFF;//控制模式
FIGHT_CONTROL fight_mode=FIGHT_ON;//战斗模式


extern int16_t SHOOT_LEFT_FRIC_SPEED_MAX;
extern int16_t SHOOT_LEFT_FRIC_SPEED_MIN;
extern int16_t SHOOT_RIGHT_FRIC_SPEED_MAX;
extern int16_t SHOOT_RIGHT_FRIC_SPEED_MIN;

extern uint8_t reboot_flag;
extern uint8_t game_status,speed_limit;

extern VISION_MODE_t vision_on_mode;

int calibrate_start_flag=0;
uint16_t last_key;
uint8_t spin_flag_1,spin_flag_2,fly_flag=0,chassis_power_flag=1;
uint8_t last_TOP_ANGLE_flag=0;
uint8_t add_flag=0;
uint8_t time_count_q=0,time_count_f=0,time_count_e=0,time_count_c=0,time_count_g=0,time_count_x=0,time_count_z=0;
float target_angle=0;
static int deadline_judge_v(int16_t a);
//死区判断
static int deadline_judge_v(int16_t a) 
{
	if(control_mode==KEY_OFF)
	{
		if(abs(a-RC_MIDD)<=DEADLINE) return 1;
	    else return 0;
	}
   	if(control_mode==KEY_ON)
	{
		if(abs(a-KEY_MIDD)<=KEY_DEADLINE) return 1;
	    else return 0;
	}
	return 1;
}
//控制模式选择
void control_mode_judge(void)
{
	if(rc_ctrl.rc.ch[0]!=0||rc_ctrl.rc.ch[1]!=0||rc_ctrl.rc.ch[2]!=0||rc_ctrl.rc.ch[3]!=0||rc_ctrl.rc.ch[4]!=0)
		control_mode=KEY_OFF;
	if(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)
		control_mode=KEY_ON;
}
int yuyuyuyu=0;
//遥控器控制模式
void remote_control_data(void)
{
	calibrate_start_flag=0;
		/*                模式选择                 */
	if(switch_is_up(rc_ctrl.rc.s[1]))  //左上   底盘运动模式选择
	{
		if(switch_is_up(rc_ctrl.rc.s[0]) && gimbal_set_mode!=GIMBAL_INIT) // 随动
		{
			gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE; //调试注释
		}
		if(switch_is_mid(rc_ctrl.rc.s[0]) && gimbal_set_mode!=GIMBAL_INIT ) // 小陀螺
		{
			gimbal_set_mode = GIMBAL_TOP_ANGLE;  //调试注释
		}
		if(switch_is_down(rc_ctrl.rc.s[0])) //  进入校准模式
		{
			gimbal_set_mode=GIMBAL_ZERO_FORCE;
//			calibrate_start_flag=1;
			return;
		}	
	}
	if(switch_is_mid(rc_ctrl.rc.s[1])) // 中 视觉选择
	{
		if(switch_is_up(rc_ctrl.rc.s[0])) // 开
		{
			fricspeed=FRIC_MAX;
			//control_mode=KEY_ON;
			chassis_power_flag=0;
		}
		else chassis_power_flag=1;
		if(switch_is_mid(rc_ctrl.rc.s[0])) // 关
		{
			vision_mode=VISION_OFF;
			fricspeed=FRIC_MIN;
		}
		if(switch_is_down(rc_ctrl.rc.s[0]))
		{
			if(vision_mode==VISION_OFF)
			{
				vision_yaw=gimbal_y.IMU_actual_angle;
			  vision_pitch=gimbal_y.IMU_actual_angle;
			}
			if(windwill_flag==1)fricspeed=FRIC_MAX;
			else fricspeed=FRIC_MIN;
      vision_mode=VISION_ON;
		}
	}
	if(switch_is_down(rc_ctrl.rc.s[1])) // 下  发子弹
	{
		if(fricspeed==FRIC_MIN)
		{
			rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
		  rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
		}
		else
		{
			rc_shoot.left_fric.target_speed =  SHOOT_LEFT_FRIC_SPEED_MAX;
		  rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MAX;
		}
		if(switch_is_down(rc_ctrl.rc.s[0]))// 单发
		{
			One_Shoot_flag=1;
			
		}
		if(switch_is_mid(rc_ctrl.rc.s[0])) // 关
		{
				One_Shoot_flag=0;
			  Ten_Shoot_flag=0;
		}
		if(switch_is_up(rc_ctrl.rc.s[0])) //  连发
		{
			Ten_Shoot_flag=1;
		}
	}
	if(deadline_judge_v(rc_ctrl.rc.ch[0])==0)
	{
		rc_sent.y_speed=limits_change(Y_SPEED_MAXX,Y_SPEED_MINN,rc_ctrl.rc.ch[0],RC_MAXX,RC_MINN);
	}
	else rc_sent.y_speed=0;
	if(deadline_judge_v(rc_ctrl.rc.ch[1])==0)
	{
		rc_sent.x_speed=limits_change(X_SPEED_MAXX,X_SPEED_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MINN);
	}
	else rc_sent.x_speed=0;
	if(deadline_judge_v(rc_ctrl.rc.ch[4])==0)
	{
		if(rc_ctrl.rc.ch[4]>0)rc_sent.r_speed=(100.0/660.0)*(float)(rc_ctrl.rc.ch[4]);//rc_ctrl.rc.ch[4]数据很怪
		else if(rc_ctrl.rc.ch[4]<0)rc_sent.r_speed=(100.0/32000.0)*(float)(rc_ctrl.rc.ch[4]);
		
		if(rc_sent.r_speed>100||rc_sent.r_speed<-100)rc_sent.r_speed=0;
	}
	else rc_sent.r_speed=0;
	
//	rc_sent.yaw.target_speed=limits_change(RC_YAW_SPEED_MAXX,RC_YAW_SPEED_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
	rc_sent.yaw.target_angle=limits_change(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
	
//	rc_sent.pitch.target_speed=limits_change(RC_PITCH_SPEED_MAXX,RC_PITCH_SPEED_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
	rc_sent.pitch.target_angle=limits_change(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
	
	
//	if(deadline_judge(rc_ctrl.rc.ch[2],1)==0)
//	{
//		rc_sent.yaw.target_speed=limits_change(RC_YAW_SPEED_MAXX,RC_YAW_SPEED_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
//		rc_sent.yaw.target_angle=limits_change(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[2],RC_MAXX,RC_MINN);
//	}
//	else
//	{
//		rc_sent.yaw.target_speed=0;
//		rc_sent.yaw.target_angle=0;
//	}
//	if(deadline_judge(rc_ctrl.rc.ch[3],2)==0)
//	{
//		rc_sent.pitch.target_speed=limits_change(RC_PITCH_SPEED_MAXX,RC_PITCH_SPEED_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
//		rc_sent.pitch.target_angle=limits_change(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[3],RC_MAXX,RC_MINN);
//	}
//	else
//	{
//		rc_sent.pitch.target_speed=0;
//		rc_sent.pitch.target_angle=0;
//	}
}
/*******************************************************键鼠控制模式 **************************************************************/

void key_control_data(void)
{
	One_Shoot_flag=0;
  Ten_Shoot_flag=0;

	//战斗模式判断
	if(KEY_board&SHIFT_key) fight_mode=RUN_AWAY;
	else fight_mode=FIGHT_ON;
	/*控制更为精细*/
	if(fight_mode==FIGHT_ON)
	{
		/*       控制云台      */
		if(gimbal_set_mode == GIMBAL_TOP_ANGLE)  //小陀螺的yaw
		{
			gimbal_y.gimbal_raw_pid.Kp=14000;
			gimbal_y.gimbal_raw_pid.max_out=25000;
//			rc_sent.yaw.target_angle=limits_change_(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,RIGHT_YAW_KEY_MAXX,LEFT_YAW_KEY_MAXX,LEFT_YAW_KEY_RANGE,RIGHT_YAW_KEY_RANGE);
		}
//        rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAX,KEY_MINN);	
		else
		{
			gimbal_y.gimbal_raw_pid.Kp=7000;
			gimbal_y.gimbal_raw_pid.max_out=13000;
		}
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAX,KEY_MINN);  //映射鼠标的值到pitch和yaw			
		rc_sent.pitch.target_angle=-limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAX,KEY_MINN);
		if(MOUSE_pre_left==1) 
		{
		  if(fricspeed==FRIC_MIN)
		  {
			  rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
		}
		  else
		  {
//			  rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
//		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
			  rc_shoot.left_fric.target_speed =  SHOOT_LEFT_FRIC_SPEED_MAX;
		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MAX;
	  	}
			One_Shoot_flag=1;
		}

		if(MOUSE_pre_right==1) 
			Ten_Shoot_flag=1;
		
		/*       控制底盘     */
		if(KEY_board&ws_key)
		{
			if((KEY_board&W_key)&&(!(KEY_board&S_key)))  //只按下w
		    {
			    rc_sent.x_speed= 100;
		    }
		  if((KEY_board&S_key)&&(!(KEY_board&W_key)))
		    {
			    rc_sent.x_speed= -100;
		    }
		  if((KEY_board&W_key)&&(KEY_board&S_key))  //同时按下 遵循最后一个按下的按键
		    {
					if((last_key&W_key)&&(!(last_key&S_key)))
						rc_sent.x_speed= -100;
			    if((last_key&S_key)&&(!(last_key&W_key)))
						rc_sent.x_speed= 100;
				}
		}
		else rc_sent.x_speed=0;
		
		if(KEY_board&ad_key)
		{
			if((KEY_board&A_key)&&(!(KEY_board&D_key)))  //只按下A
		    {
			    rc_sent.y_speed=-80;
		    }
		  if((KEY_board&D_key)&&(!(KEY_board&A_key)))
		    {
			    rc_sent.y_speed= 80;
		    }
			if((KEY_board&A_key)&&(KEY_board&D_key))  //同时按下 遵循最后一个按下的按键
		    {
					if((last_key&A_key)&&(!(last_key&D_key)))
						rc_sent.y_speed= 80;
			    if((last_key&D_key)&&(!(last_key&A_key)))
						rc_sent.y_speed= -80;
				}
		}
		else rc_sent.y_speed=0;


//		if((KEY_board&X_key)&&(vision_mode==VISION_ON))  //按下X开启自瞄程序重启
//		{
//			reboot_flag=1;
//		}
//		else reboot_flag=0;
	}
	
   	if(fight_mode==RUN_AWAY)
	{
			/*       控制云台      */
//		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_RUN,KEY_YAW_ANGLE_MINN_RUN,MOUSE_x,YAW_KEY_MAXX,KEY_MINN);
////		rc_sent.yaw.target_angle=limits_change_(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,YAW_KEY_MAXX,KEY_MINN);  //映射鼠标的值到pitch和yaw
//		//if(deadline_judge(MOUSE_y,2)==0)
//		rc_sent.pitch.target_angle=-limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_y,PITCH_KEY_MAXX,KEY_MINN);
	  
		if(gimbal_set_mode == GIMBAL_TOP_ANGLE)  //小陀螺的yaw
		{
//			gimbal_y.gimbal_raw_pid.Kp=14000;  //提高小陀螺精度
			gimbal_y.gimbal_raw_pid.max_out=20000;
//			rc_sent.yaw.target_angle=limits_change_(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,RIGHT_YAW_KEY_MAXX,LEFT_YAW_KEY_MAXX,LEFT_YAW_KEY_RANGE,RIGHT_YAW_KEY_RANGE);
		}	
		else
		{
			gimbal_y.gimbal_raw_pid.Kp=12000;  //正常模式的参数
			gimbal_y.gimbal_raw_pid.max_out=13000;
		}
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAX,KEY_MINN);  //映射鼠标的值到pitch和yaw			
		rc_sent.pitch.target_angle=-limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAX,KEY_MINN);
		
		if(MOUSE_pre_left==1) 
		{
		  if(fricspeed==FRIC_MIN)
		  {
			  rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
		  }
		  else
		  {
//				rc_shoot.left_fric.target_speed = SHOOT_LEFT_FRIC_SPEED_MIN;
//		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MIN;
			  rc_shoot.left_fric.target_speed =  SHOOT_LEFT_FRIC_SPEED_MAX;
		    rc_shoot.right_fric.target_speed = SHOOT_RIGHT_FRIC_SPEED_MAX;
		  }
			One_Shoot_flag=1;
		}
		if(MOUSE_pre_right==1) Ten_Shoot_flag=1;
//						rc_shoot.left_fric.target_speed = -SHOOT_FRIC_SPEED_MIN;
//		  	rc_shoot.right_fric.target_speed = SHOOT_FRIC_SPEED_MIN;
		/*       控制底盘     */
		if(KEY_board&ws_key)
		{
			if(KEY_board&W_key)
		    {
			    rc_sent.x_speed= 200;
		    }
		    if(KEY_board&S_key)
		    {
			    rc_sent.x_speed= -200;
		    }
		}
		else rc_sent.x_speed=0;

		if(KEY_board&ad_key)
		{
			if(KEY_board&A_key)
		    {
			    rc_sent.y_speed=-170;
		    }
		    if(KEY_board&D_key)
		    {
			    rc_sent.y_speed= 170;
		    }
		}
		else rc_sent.y_speed=0;


	
//		if((KEY_board&X_key)&&(vision_mode==VISION_ON))  //按下X开启自瞄程序重启
//		{
//			reboot_flag=1;
//		}
//		else reboot_flag=0;
	}
}

uint8_t flag__=0;
void judge_q(void)  //开关小陀螺
{
		if(KEY_board&Q_key)
			time_count_q++;
		else
		{
			if(last_key&Q_key)
			{
				if(time_count_q>=KEY_COUNT) //7*14ms按下时间
			   {
			   	 if(gimbal_set_mode==GIMBAL_ABSOLUTE_ANGLE)
						 gimbal_set_mode = GIMBAL_TOP_ANGLE;	//小陀螺					
				   else
						gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;		
//				if(flag__==0)
//					flag__=1;
//				else 
//					flag__=0;			 
			   }	
			}
			time_count_q=0;  //计数清零
		}			
}

void judge_f(void)  //开关自瞄
{
	if(KEY_board&F_key)
		time_count_f++;
	else 
	{
		if(last_key&F_key)
		{
			if(time_count_f>=KEY_COUNT)
		   {
			   if(vision_mode==VISION_OFF)
			    {
				  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  //开自瞄关闭激光
						vision_yaw=gimbal_y.CAN_actual_angle;
			      vision_pitch=gimbal_p.IMU_actual_angle;
			      vision_mode=VISION_ON;
            vision_on_mode=AUTO_ON;  //开启自瞄默认自瞄模式是装甲板自瞄						
					}
			   else
			    {
				  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  //关自瞄开启激光
						vision_mode=VISION_OFF;				
			    }
		   }
		}
		time_count_f=0;
	}
}

void judge_g(void) //开启自喵的情况下三种自瞄模式来回切换
{
	if(KEY_board&G_key)
		time_count_g++;
	else
	{
    if(last_key&G_key)
		{
			if(time_count_g>=KEY_COUNT)  
		  {  
				if(vision_mode==VISION_ON)
				{
					if(vision_on_mode==AUTO_ON)
						vision_on_mode=WINDWILL_SMALL;
					if(vision_on_mode==WINDWILL_SMALL)
						vision_on_mode=WINDWILL_BIG;
					if(vision_on_mode==WINDWILL_BIG)
						vision_on_mode=AUTO_ON;					
				}
		  }			
		}			
		time_count_g=0;		
	}		
}

extern uint16_t set_compare;
void judge_e(void)  //开关弹舱盖
{
	if(KEY_board&E_key)
		time_count_e++;
	else 
	{
		if(last_key&E_key)
		{
			if(time_count_e>=KEY_COUNT)
		   { 
			   if(set_compare==700)  
					 set_compare=1200;
			   else 
					 set_compare=700;
		   }
		}
		time_count_e=0;
	}
}

void judge_c(void) //180°掉头
{
	if(KEY_board&C_key)
		time_count_c++;
	else
	{
    if(last_key&C_key)
		{
			if(time_count_c>=KEY_COUNT)
		  {  
		  	gimbal_y.target_angle+=180;
		  }			
		}			
		time_count_c=0;		
	}		
}

//void judge_v(void)
//{
//	if(KEY_board&V_key)
//		time_count_v++;
//	else
//	{
//		if(last_key&V_key)
//		{
//			if(time_count_v>=KEY_COUNT)
//			{
//				if(flag__==0)
//					flag__=1;
//				else 
//					flag__=0;
//			}
//		}
//		time_count_v=0;		
//	}
//}

//static void judge_x(void)
//{
//	if(KEY_board&X_key)
//		time_count_v++;
//	else
//	{
//		if(vision_mode==VISION_ON)
//		{
//			if(last_key&X_key)
//			{
//				if(time_count_v>=KEY_COUNT)
//				{
//					if(reboot_flag==0)
//						reboot_flag=1;
//					else 
//						reboot_flag=0;
//				}
//			}
//		}
//		time_count_v=0;		
//	}
//}
static void judge_x(void)
{
	if(KEY_board&X_key)
	{
		time_count_x++;
		if(time_count_x>800) time_count_x-=100;
		if(vision_mode==VISION_ON)
		{
			  if(time_count_x>=LONG_KEY_COUNT)  //长按1s断开上位机继电器
			  {
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
					reboot_flag=1;
			  }
		}	
	}
	else
	{
	 time_count_x=0;		
	}
		
}

uint8_t supercap_reboot_flag;
void judge_z(void)
{
	if(KEY_board&Z_key)
	{
		time_count_z++;
		if(time_count_z>=300) time_count_z=300;	
	}
	else
	{ 
		if(time_count_z>=LONG_KEY_COUNT-40)
		{
		if( supercap_reboot_flag==0)	
		supercap_reboot_flag=1;
		else
		supercap_reboot_flag=0;
		
		time_count_z=0;
		}
   
	}			
}

int shoot_close_flag=0;
int shoot_trigger_cnt_flag=0;


uint16_t switch_cnt=0;
void upper_computer_reboot(void)
{

		if(reboot_flag)
		{
			switch_cnt++;
			if(switch_cnt>=2500)
			{
				switch_cnt=0;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
				reboot_flag=0;
			}
		}
}
void judge_key(void)
{
  judge_z();
	judge_c();
	judge_x();
	judge_e();
	judge_f();
	judge_q();
	judge_g();
	last_key=KEY_board;
}
