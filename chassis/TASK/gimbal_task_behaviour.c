#include "gimbal_task_behaviour.h"
#include "gimbal_task.h"
#include "bsp_math.h"
#include "vision_task.h"

#define yaw_angle gimbal_y.add_angle
#define pitch_angle gimbal_p.add_angle


static void gimbal_zero_force_control(float *yaw,float *pitch);
static void gimbal_init_control(float *yaw,float *pitch);
static void IMU_yaw_angle_limit(void);
static void CAN_yaw_angle_limit(void);
static void IMU_pitch_angle_limit(void);
static void CAN_pitch_angle_limit(void);
static int deadline_judge(float a,int flag);
//yaw 1 pitch 2
/**
  * @breif         死区判断
  * @param[in]     目标角度a和yaw,pitch的标志位flag
	* @param[out]    none
  * @retval        在死区中时返回0,否则返回1     
  */
static int deadline_judge(float a,int flag)
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

/**
  * @breif         云台控制行为,控制云台的pitch,yaw运动
  * @param[in]     none
	* @param[out]    pitch,yaw的目标角度
  * @retval        none     
  */
int PR_Pitch_Flag=0,PR_Yaw_Flag;

void gimbal_control_behaviour(void)
{
	
	/*         增量        */
	if(vision_mode==VISION_OFF)
	{
		//开启巡航模式
		if(gimbal_prtrol==PATROL_ON)
		{
			//pitch,yaw开始上下和左右摆动
			gimbal_y.target_angle=gimbal_y.IMU_actual_angle-yaw_angle;
			if(PR_Pitch_Flag==0)
			{
				gimbal_p.target_angle=gimbal_p.IMU_actual_angle+pitch_angle;
				if(gimbal_p.target_angle>35.0f)
				{PR_Pitch_Flag=1;gimbal_p.target_angle=35.0f;}
			}
			if(PR_Pitch_Flag==1)
			{
				gimbal_p.target_angle=gimbal_p.IMU_actual_angle-pitch_angle;
				if(gimbal_p.target_angle<10.0f)
				{PR_Pitch_Flag=0;gimbal_p.target_angle=10.0f;}
			}
		}
		else
		{
			//pitch,yaw可自由调试
			if(deadline_judge(yaw_angle,1)!=0)
	        gimbal_y.target_angle=gimbal_y.IMU_actual_angle-yaw_angle;
	        if(deadline_judge(pitch_angle,2)!=0)
	        gimbal_p.target_angle=gimbal_p.IMU_actual_angle+pitch_angle;
		}
		
	}
	else
	{
		//开启巡航模式
		if(gimbal_prtrol==PATROL_ON)
		{
			//pitch,yaw开始上下和左右摆动
			if(PR_Yaw_Flag==0)
			{
				gimbal_y.target_angle=gimbal_y.IMU_actual_angle+yaw_angle;
				if(gimbal_y.target_angle>80.0f)
				{PR_Yaw_Flag=1;gimbal_y.target_angle=80.0f;}
			}
			if(PR_Yaw_Flag==1)
			{
				gimbal_y.target_angle=gimbal_y.IMU_actual_angle-yaw_angle;
				if(gimbal_y.target_angle<-20.0f)
				{PR_Yaw_Flag=0;gimbal_y.target_angle=-20.0f;}
			}
			if(PR_Pitch_Flag==0)
			{
				gimbal_p.target_angle=gimbal_p.IMU_actual_angle+pitch_angle;
				if(gimbal_p.target_angle>40.0f)
				{PR_Pitch_Flag=1;gimbal_p.target_angle=40.0f;}
			}
			if(PR_Pitch_Flag==1)
			{
				gimbal_p.target_angle=gimbal_p.IMU_actual_angle-pitch_angle;
				if(gimbal_p.target_angle<4.0f)
				{PR_Pitch_Flag=0;gimbal_p.target_angle=4.0f;}
			}
		}
		else
		{
			//pitch,yaw可自由调试
			gimbal_y.target_angle = yaw_angle;
	    	gimbal_p.target_angle = pitch_angle;
		}
		
	}	
	//限制在-180到+180之间
	if(gimbal_y.target_angle>180)
		gimbal_y.target_angle -= 360;
	if(gimbal_y.target_angle<-180)
		gimbal_y.target_angle += 360;

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



