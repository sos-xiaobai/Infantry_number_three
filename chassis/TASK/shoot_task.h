#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "pid.h"

#define SHOOT_NUM 1
#define SHOOT_FRIC_SPEED 6500   //5000


typedef struct{
	
	float target_speed;
	float actual_speed;
	
	float target_angle;
	float actual_angle;
	
	int last_shoot_flag;
	int last_back_flag;
	float set_currunt;
	
	float last_angle;			//上一次机械转子角度
	int rounds;				    //转过的圈数
	int total_angle;			//总共转过的角度
	int last_speed;       		//上一次真实转速
	int record_begin_angle_status;//是否记录了电机初始角度 0代表没有记录，1代表记录成功
	int begin_angle;            //电机初始角度
	
	PidTypeDef speed_pid;
	PidTypeDef angle_pid;
}trigger_t;

typedef struct{
	
	float target_speed;
	float actual_speed;
	
	float set_currunt;
	

	PidTypeDef speed_pid;
}fric_t;


typedef struct{
	
	fric_t left_fric;
	fric_t right_fric;
	
	trigger_t trigger;
}shoot_task_t;

void shoot_init(void);
void shoot_task(void);
void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed);
extern shoot_task_t rc_shoot;
extern int One_Shoot_flag;
extern int Ten_Shoot_flag;
#endif
