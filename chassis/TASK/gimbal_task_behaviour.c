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
  * @breif         �����ж�
  * @param[in]     Ŀ��Ƕ�a��yaw,pitch�ı�־λflag
	* @param[out]    none
  * @retval        ��������ʱ����0,���򷵻�1     
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
  * @breif         ��̨������Ϊ,������̨��pitch,yaw�˶�
  * @param[in]     none
	* @param[out]    pitch,yaw��Ŀ��Ƕ�
  * @retval        none     
  */
int PR_Pitch_Flag=0,PR_Yaw_Flag;

void gimbal_control_behaviour(void)
{
	
	/*         ����        */
	if(vision_mode==VISION_OFF)
	{
		//����Ѳ��ģʽ
		if(gimbal_prtrol==PATROL_ON)
		{
			//pitch,yaw��ʼ���º����Ұڶ�
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
			//pitch,yaw�����ɵ���
			if(deadline_judge(yaw_angle,1)!=0)
	        gimbal_y.target_angle=gimbal_y.IMU_actual_angle-yaw_angle;
	        if(deadline_judge(pitch_angle,2)!=0)
	        gimbal_p.target_angle=gimbal_p.IMU_actual_angle+pitch_angle;
		}
		
	}
	else
	{
		//����Ѳ��ģʽ
		if(gimbal_prtrol==PATROL_ON)
		{
			//pitch,yaw��ʼ���º����Ұڶ�
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
			//pitch,yaw�����ɵ���
			gimbal_y.target_angle = yaw_angle;
	    	gimbal_p.target_angle = pitch_angle;
		}
		
	}	
	//������-180��+180֮��
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
//��Ūpitch��Ūyaw
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



