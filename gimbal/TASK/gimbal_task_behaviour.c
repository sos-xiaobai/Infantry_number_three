#include "gimbal_task_behaviour.h"
#include "gimbal_task.h"
#include "bsp_math.h"
#include "vision_task.h"
#include "rc_task.h"
#include <math.h>
#define yaw_angle gimbal_y.add_angle
#define pitch_angle gimbal_p.add_angle

extern uint8_t traget_exit_flag;

extern uint8_t reboot_flag;
float chassis_pitch_angle;
float chassis_pitch_speed;
float chassis_yaw_angle;
float chassis_yaw_speed;
int8_t chassis_imu_data[8];
extern uint8_t vision_check_flag;

static void gimbal_control_behaviour_mode(void);
static void gimbal_zero_force_control(float *yaw,float *pitch);
static void gimbal_init_control(float *yaw,float *pitch);
static void IMU_yaw_angle_limit(void);
static void CAN_yaw_angle_limit(void);
static void IMU_pitch_angle_limit(void);
static void CAN_pitch_angle_limit(void);
void fly_pitch_judge(void);
void chassis_imu_data_get(void);
//static int deadline_judge(float a,int flag);
//yaw 1 pitch 2
int deadline_judge(float a,int flag)
{
	if(flag==2)
	{
		if(abs(a)<=PITCH_DEADLINE) return 0;
	    else return 1;
	}
	else
	{
		if(abs(a)<=YAW_DEADLINE) return 0;
	    else return 1;		
	}

}


void gimbal_control_behaviour(void)
{
	gimbal_control_behaviour_mode();
	/*         增量        */
	if(vision_mode==VISION_OFF)
	{
		if(deadline_judge(yaw_angle,1)!=0)
		{
			if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
			{
	    gimbal_y.target_angle=gimbal_y.CAN_actual_angle-yaw_angle;
			}
			if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
			{
			gimbal_y.target_angle=gimbal_y.IMU_actual_angle-yaw_angle;
			}
		}
	    if(deadline_judge(pitch_angle,2)!=0)
				gimbal_p.target_angle=gimbal_p.IMU_actual_angle+pitch_angle; 				//pitch_angle就是add_angle
			chassis_imu_data_get();
			fly_pitch_judge();
//    gimbal_p.target_angle=5;
//    if(add_flag==1)
//	  {
//		  gimbal_y.target_angle=target_angle;
//		  if(fabs(target_angle-gimbal_y.target_angle)<2.0f)
//		  {
//			  add_flag=0;
//		  }
//	  }
  }
	else
	{
		if(traget_exit_flag)    //上位机获得操控权
		{
			if(vision_check_flag==0) return;
			gimbal_y.target_angle = yaw_angle;
			gimbal_p.target_angle = pitch_angle;
		}
		else 
		{
			if(vision_check_flag==1) return;  //不处理切换控制权的这一帧
			if(deadline_judge(yaw_angle,1)!=0)
			{
				if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
				{
				gimbal_y.target_angle=gimbal_y.CAN_actual_angle-yaw_angle;
				}
				if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
				{
				gimbal_y.target_angle=gimbal_y.IMU_actual_angle-yaw_angle;
				}
			}
				if(deadline_judge(pitch_angle,2)!=0)
				  gimbal_p.target_angle=gimbal_p.IMU_actual_angle+pitch_angle;
				chassis_imu_data_get();
				fly_pitch_judge();
		}
	}	
	
	if(gimbal_p.target_angle<-15) //限制pitch和yaw的角度范围
	gimbal_p.target_angle=-15;
	if(gimbal_p.target_angle>30)
	gimbal_p.target_angle=30;

}

void fly_pitch_judge(void)
{
		 static uint8_t fly_flag=0;
	   static float last_pitch_angle1=0;
     if(((chassis_pitch_angle>=15) || (chassis_pitch_angle<=-15))&&fly_flag==0)  //触发飞坡并且只进入一次
			{
				if(chassis_pitch_angle>0)
					gimbal_p.target_angle=pitch_angle+chassis_pitch_angle+15;
				else
					gimbal_p.target_angle=pitch_angle+chassis_pitch_angle-5;
				last_pitch_angle1=chassis_pitch_angle;
				fly_flag=1;
			}
			else if(((chassis_pitch_angle<=5) && (chassis_pitch_angle>=-5))&&fly_flag==1)  //结束飞坡
			{
				if(last_pitch_angle1>0)
					gimbal_p.target_angle=pitch_angle+gimbal_p.IMU_actual_angle-15;
				else
					gimbal_p.target_angle=pitch_angle+gimbal_p.IMU_actual_angle+15;
				fly_flag=0;
			}
			else if(deadline_judge(pitch_angle,2)!=0) 
			{
				if(fly_flag==1)  //飞坡过程中的限位
				{
				  gimbal_p.target_angle=pitch_angle+gimbal_p.IMU_actual_angle;	
          if(gimbal_p.target_angle<=-15+chassis_pitch_angle) 	gimbal_p.target_angle=-15+chassis_pitch_angle;
          if(gimbal_p.target_angle>= 30+chassis_pitch_angle) 	gimbal_p.target_angle= 30+chassis_pitch_angle;					 
				}
				else  //平地限位
				{
					gimbal_p.target_angle=pitch_angle+gimbal_p.IMU_actual_angle;	
          if(gimbal_p.target_angle<=-15) 	gimbal_p.target_angle=-15;
          if(gimbal_p.target_angle>= 30) 	gimbal_p.target_angle= 30;							 
				}
		   }	
}

void chassis_imu_data_get(void)
{
		  chassis_yaw_angle=(float)(chassis_imu_data[0]<<8|(chassis_imu_data[1]&0xff))/100.0f;
		  chassis_yaw_speed=(float)(chassis_imu_data[2]<<8|(chassis_imu_data[3]&0xff))/100.0f;
		  chassis_pitch_angle=(float)(chassis_imu_data[4]<<8|(chassis_imu_data[5]&0xff))/100.0f;
		 	chassis_pitch_speed=(float)(chassis_imu_data[6]<<8|(chassis_imu_data[7]&0xff))/100.0f;
}

static void gimbal_control_behaviour_mode(void)
{
	if(gimbal_set_mode==GIMBAL_ZERO_FORCE)
	{
		gimbal_zero_force_control(&yaw_angle,&pitch_angle);
	}
	if(gimbal_set_mode==GIMBAL_INIT)
	{
		gimbal_init_control(&yaw_angle,&pitch_angle);
	}
}


static void gimbal_zero_force_control(float *yaw,float *pitch)
{
	if(yaw == NULL || pitch == NULL)
	{
		return;
	}
	*yaw = 0;
	*pitch = 0;
}
//先弄pitch再弄yaw
static void gimbal_init_control(float *yaw,float *pitch)
{
	if(yaw == NULL || pitch == NULL)
	{
		return;
	}
	
	if (fabs(INIT_PITCH_SET - gimbal_p.IMU_actual_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_p.IMU_actual_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_p.IMU_actual_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_y.IMU_actual_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

//一下都是没用到的函数
static void IMU_yaw_angle_limit()
{
	float angle_basic=gimbal_y.target_angle-gimbal_y.IMU_actual_angle;
	if(gimbal_y.IMU_actual_angle+angle_basic+yaw_angle>YAW_ANGLE_MAX)
		if(yaw_angle>0.0f)
		yaw_angle=YAW_ANGLE_MAX-gimbal_y.IMU_actual_angle-angle_basic;
	if(gimbal_y.IMU_actual_angle+angle_basic+yaw_angle<YAW_ANGLE_MIN)
		if(yaw_angle<0.0f)
			yaw_angle=YAW_ANGLE_MIN-gimbal_y.IMU_actual_angle-angle_basic;
}

static void CAN_yaw_angle_limit()
{
	float angle_basic=gimbal_y.target_angle-gimbal_y.CAN_actual_angle;
	if(gimbal_y.IMU_actual_angle+angle_basic+yaw_angle>YAW_ANGLE_MAX)
		if(yaw_angle>0.0f)
		yaw_angle=YAW_ANGLE_MAX-gimbal_y.CAN_actual_angle-angle_basic;
	if(gimbal_y.CAN_actual_angle+angle_basic+yaw_angle<YAW_ANGLE_MIN)
		if(yaw_angle<0.0f)
			yaw_angle=YAW_ANGLE_MIN-gimbal_y.CAN_actual_angle-angle_basic;
}


static void IMU_pitch_angle_limit()
{
	float angle_basic=gimbal_p.target_angle-gimbal_p.IMU_actual_angle;
	if(gimbal_p.IMU_actual_angle+angle_basic+pitch_angle>YAW_ANGLE_MAX)
		if(pitch_angle>0.0f)
		pitch_angle=YAW_ANGLE_MAX-gimbal_p.IMU_actual_angle-angle_basic;
	if(gimbal_p.IMU_actual_angle+angle_basic+pitch_angle<YAW_ANGLE_MIN)
		if(pitch_angle<0.0f)
			pitch_angle=YAW_ANGLE_MIN-gimbal_p.IMU_actual_angle-angle_basic;
}

static void CAN_pitch_angle_limit()
{
	float angle_basic=gimbal_p.target_angle-gimbal_p.CAN_actual_angle;
	if(gimbal_p.CAN_actual_angle+angle_basic+pitch_angle>YAW_ANGLE_MAX)
		if(pitch_angle>0.0f)
		pitch_angle=YAW_ANGLE_MAX-gimbal_p.CAN_actual_angle-angle_basic;
	if(gimbal_p.CAN_actual_angle+angle_basic+pitch_angle<YAW_ANGLE_MIN)
		if(pitch_angle<0.0f)
			pitch_angle=YAW_ANGLE_MIN-gimbal_p.CAN_actual_angle-angle_basic;
}



