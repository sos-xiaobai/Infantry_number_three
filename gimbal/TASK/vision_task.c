#include "vision_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "bsp_math.h"
#include "chassis_task.h"
#include <math.h>
VISION_t vision_mode=VISION_OFF;
VISION_GET_t vision_sent;
VISION_MODE_t vision_on_mode;
float chassis_angle=0;//�����ٶ��������̨�ο�ϵ�ļн�
float vision_x=0,vision_y=0;//�����ٶ�����̨����ϵ�µ��ٶ�
float vision_yaw=0,vision_pitch=0; //����λ��
int vision_cnt=0;
extern uint8_t traget_exit_flag;
extern uint8_t auto_exposure_flag;
extern uint8_t reboot_flag;


int vision_delay(int vt)
{
	vision_cnt++;
	if(vision_cnt>vt)  
	{
		vision_cnt=0;
	    return 1;
	}
	else return 0;
}
int yu=0;
float last_yaw_target_angle=0,last_pitch_target_angle=0;
float yaw_deadline_angle=0.01f,pitch_deadline_angle=0.01;  //����
uint8_t vision_check_flag=0;  //���ڷ�ֹ�������Ȩ�л�ʱ��������
void Vision_Task(void)
{
	if(vision_mode==VISION_OFF)          //����ģʽ��
	{
		
		if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
		{
	        gimbal_y.target_speed=((rc_sent.yaw.target_angle)/5);
		}
		if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
		{
	        gimbal_y.add_angle=(8192.0f/360.0f)*(rc_sent.yaw.target_angle);
		}
		if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
		{
			gimbal_y.add_angle=rc_sent.yaw.target_angle;
		}
		
		
//		gimbal_y.add_angle=rc_sent.yaw.target_angle;
		gimbal_p.add_angle=rc_sent.pitch.target_angle;
//		gimbal_y.target_speed=2*(rc_sent.yaw.target_angle);
//		gimbal_y.target_speed=((rc_sent.yaw.target_angle)/4.5);
//		gimbal_y.target_angle=Ren;
	}
	else  //����ģʽ��
	{
		if(traget_exit_flag==1)   //�Ӿ������ٿ�Ȩ����λ��
		{
			vision_check_flag=1;
			if(vision_sent.yaw.target_angle-last_yaw_target_angle>yaw_deadline_angle||vision_sent.yaw.target_angle-last_yaw_target_angle<-yaw_deadline_angle)  //����
			{
//			gimbal_y.add_angle=vision_sent.yaw.target_angle;
//			last_yaw_target_angle=vision_sent.yaw.target_angle;
//			gimbal_y.add_angle=(vision_sent.yaw.target_angle-imu_can_error_y);
//			last_yaw_target_angle=(vision_sent.yaw.target_angle-imu_can_error_y);
				if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
				{
					gimbal_y.add_angle=(8192.0f/360.0f)*(vision_sent.yaw.target_angle-imu_can_error_y);
					last_yaw_target_angle=(8192.0f/360.0f)*(vision_sent.yaw.target_angle-imu_can_error_y);
				}
				if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
				{
					gimbal_y.add_angle=vision_sent.yaw.target_angle;
					last_yaw_target_angle=vision_sent.yaw.target_angle;
				}
			}
			if(vision_sent.pitch.target_angle-last_pitch_target_angle>pitch_deadline_angle||vision_sent.pitch.target_angle-last_pitch_target_angle<-pitch_deadline_angle)
			{
				gimbal_p.add_angle=vision_sent.pitch.target_angle;
				last_pitch_target_angle=vision_sent.pitch.target_angle;
			}
		}
		else   //�Ӿ��������ǲٿ�Ȩ����λ��
		{
			vision_check_flag=0;
			if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
			{
				gimbal_y.target_speed=((rc_sent.yaw.target_angle)/5);
			}
			if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
			{
				gimbal_y.add_angle=(8192.0f/360.0f)*(rc_sent.yaw.target_angle);
			}
			if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
			{
				gimbal_y.add_angle=rc_sent.yaw.target_angle;
			}
			gimbal_p.add_angle=rc_sent.pitch.target_angle;  //ǿ��Ϊimu����ģʽ
		}
	}
}


