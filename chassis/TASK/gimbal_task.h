#ifndef __gimbal_task_H
#define __gimbal_task_H

#include "pid.h"
#include "bsp_math.h"
#include "remote_control.h"
#include "stm32f4xx.h"
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 3.7f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000

//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 320.0f

//云台角度范围
#define YAW_ANGLE_MAX 360.0f
#define YAW_ANGLE_MIN 0.0f

#define PITCH_ANGLE_MAX 360.0f
#define PITCH_ANGLE_MIN 0.0f

#define PRTROL_ANGKLE_YAW 4.0
#define PRTROL_ANGKLE_PITCH 3

typedef enum{
	VISION_ON_ALL= 0, //全自动控制模式
    VOLUNTARIY_PATROL,    //自动巡航模式
    VISION_OFF_ALL, //调试模式
}GIMBAL_MODE_t;

typedef enum{
	GIMBAL_MOTOR_RAW= 0, //电机陀螺仪速度环控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
}MODE_t;

typedef struct{
	
	PidTypeDef gimbal_raw_pid;
	PidTypeDef gimbal_gyro_pid;
	PidTypeDef gimbal_enconde_pid_speed;//速度
	PidTypeDef gimbal_enconde_pid;//角度
	//位置式
	float target_angle;
	float IMU_actual_angle;
	   
	float target_speed;
	float IMU_actual_speed;
	
	float CAN_actual_speed;
	float CAN_actual_angle;
	float CAN_total_angle;
	
	float add_angle;
	
	int16_t given_current;
	
	MODE_t gimbal_motor_mode;
}GIMBAL_t;

extern GIMBAL_t gimbal_y,gimbal_p;
extern GIMBAL_MODE_t gimbal_set_mode;
void Gimbal_Task(void);
void Gimbal_Init(void);

#endif
