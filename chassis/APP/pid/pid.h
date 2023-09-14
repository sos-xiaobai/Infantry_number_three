#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include <stdlib.h>


//角度环宏定义
#define aPID_OUT_MAX          150		//即最大速度
#define aPID_OUT_MAX_CHASSIS          50		//即最大速度
#define aPID_OUT_MAX_FOLLOW         20		//即最大速度
#define aP_OUT_MAX_FOLLOW          	20
#define FOLLOW_Integral_amax         1// 3000
#define FOLLOW_aIntegralSeparation    1//20

//速度环宏定义
//#define CHASSIS_Integral_max         5000            //抗积分饱和
#define CHASSIS_IntegralSeparation   500             //积分分离
//#define CHASSIS_vPID_max             4000            //输出限幅
extern int CHASSIS_vPID_max;
#define FOLLOW_Integral_max         500// 3000
#define FOLLOW_IntegralSeparation    15//20f
#define FOLLOW_vPID_max              3000
#define AVERAGE                      10              //平均误差数组长度 

//功率环宏定义
#define POWER_Integral_max         180// 3000
#define POWER_IntegralSeparation    12//20
#define PID_OUT_MAX_POWER          2000		//即最大速度
#define P_OUT_MAX_POWER          	2000


#define P_OUT_MAX_BUFFER   1000
#define POSITION_LOOP     1
#define SPEED_LOOP     2

#define pid_init() \
				do{ \
					/*底盘位置环*/                        \
					chassis_center.pid.position_loop.pid_Parameter.Akp = 0.07; \
					chassis_center.pid.position_loop.pid_Parameter.Aki = 0; \
					chassis_center.pid.position_loop.pid_Parameter.Akd = 0.0; \
					chassis_center.pid.position_loop.pid_Parameter.Vkp = 0; \
					chassis_center.pid.position_loop.pid_Parameter.Vki = 0; \
					chassis_center.pid.position_loop.pid_Parameter.Vkd = 0; \
					                                     \
					/*底盘速度环*/                        \
					chassis_center.pid.speed_loop.pid_Parameter.Vkp = 0.6; \
					chassis_center.pid.speed_loop.pid_Parameter.Vki = 0.0; \
					chassis_center.pid.speed_loop.pid_Parameter.Vkd = 0.2; \
					                   \
					/*轮子速度环*/       \
					chassis_motor1.pid.speed_loop.pid_Parameter.Vkp = 8;   \
					chassis_motor2.pid.speed_loop.pid_Parameter.Vkp = 8;   \
					chassis_motor3.pid.speed_loop.pid_Parameter.Vkp = 8;   \
					chassis_motor4.pid.speed_loop.pid_Parameter.Vkp = 8;   \
					chassis_motor1.pid.speed_loop.pid_Parameter.Vki = 0.05;   \
					chassis_motor2.pid.speed_loop.pid_Parameter.Vki = 0.05;   \
					chassis_motor3.pid.speed_loop.pid_Parameter.Vki = 0.05;   \
					chassis_motor4.pid.speed_loop.pid_Parameter.Vki = 0.05;   \
					chassis_motor1.pid.speed_loop.pid_Parameter.Vkd = 5;   \
					chassis_motor2.pid.speed_loop.pid_Parameter.Vkd = 5;   \
					chassis_motor3.pid.speed_loop.pid_Parameter.Vkd = 5;   \
					chassis_motor4.pid.speed_loop.pid_Parameter.Vkd = 5;   \
					     \
					/*功率环*/   \
					p_pid.kp=4.6; \
					p_pid.ki=1.8; \
					p_pid.kd=0; \
					/*功率环*/   \
					b_pid.kp=5.1; \
					b_pid.ki=1.2; \
					b_pid.kd=0; \
				}while(0)

typedef enum
{
	
	CHASSIS = 1,
	FOLLOW =2,
	NUL=0,
}switch_flag_t;


typedef struct{
	
	float err;
	float last_err;
	float err_integration;
	float actual_angle;
	float target_angle;
	float P_OUT;
	float I_OUT;
	float D_OUT;
	float PID_OUT;
	float last_actual_angle;
}APID_t;

typedef struct{
	int err;
	int last_err;
	int err_integration;
	int target_speed;
	int actual_speed;
	
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
	
	int pid_count;           //pid变量，用于计算平均误差
	float average_err;           //平均误差，使误差曲线更加平滑
	float last_average_err;      //上一次平均误差
}VPID_t;

typedef struct
{
	float Akp;			
	float Aki;	
  float Akd;	
	float Vkp;			
	float Vki;	
  float Vkd;	
}Parameter_t;

typedef struct
{
	APID_t apid;
	VPID_t vpid;
	Parameter_t pid_Parameter;
}PID_Loop_t;

typedef struct{
	uint8_t loop_flag;
	PID_Loop_t position_loop;
	PID_Loop_t speed_loop;
}PID_t;

typedef struct{
	float err;
	float last_err;
	float err_integration;
	float actual_power;
	float target_power;
	float P_OUT;
	float I_OUT;
	float D_OUT;
	float PID_OUT;
	float last_actual_power;
	float kp;			
	float ki;	
  float kd;
}POWER_PID_t;


typedef struct{
		float err;
	float last_err;
	float err_integration,err_integration_max;
	float err_max;
	float actual_buffer;
	float target_buffer;
	float P_OUT;
	float I_OUT;
	float D_OUT;
	float PID_OUT;
	float last_actual_buffer;
	float kp;			
	float ki;	
  float kd;
}BUFFER_PID_t;

typedef struct
{
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
		float deadband;
	
    float ref;
    float fdb;
	float lastfdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;

extern switch_flag_t switch_flag;
float PID_Calc(PidTypeDef *pid,  float ref,float fdb);
void PID_Init(PidTypeDef *pid,  const float PID[6]);
void PID_clear(PidTypeDef *pid);
void motor_pid_init(PID_t *pid);
void pid_realize(PID_t *pid);
void POWER_PID_Init(POWER_PID_t *pid);
void power_pid_realize(POWER_PID_t *pid);
void BUFFER_PID_Init(BUFFER_PID_t *pid);
void buffer_pid_realize(BUFFER_PID_t *pid);
#endif
