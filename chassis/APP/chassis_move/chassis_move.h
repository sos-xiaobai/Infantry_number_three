#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "kinematics.h"
#include "can.h"
#include "referee.h"
#include <math.h>

#define CHASSIS_NO_FORCE   3
#define CHASSIS_FOLLOW   	 1
#define CHASSIS_NORMAL     0
#define CHASSIS_SPIN       2

#define CLOCKWISE          1
#define ANTICLOCKWISE      2

//#define GIMBAL_HEAD_ANGLE  31    //270  //245   //161  342

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f 
#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
//#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
//#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

#define K_M      0.3f
//#define POWER_K1 0
//#define POWER_K2 0


#define follow_pid_realize() \
				do{ \
						switch_flag=FOLLOW;      \
						pid_realize(&(chassis_center.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				
#define vpid_chassis_realize() \
				do{ \
						switch_flag=CHASSIS;      \
						pid_realize(&(chassis_motor1.pid));   \
						pid_realize(&(chassis_motor2.pid));   \
						pid_realize(&(chassis_motor3.pid));   \
						pid_realize(&(chassis_motor4.pid));   \
					  switch_flag=NUL;  \
				}while(0)

//#define chassis_spin(vx,vy) \
//				do{ \
//					float theta;       \
//					theta=Get_chassis_theta();   \
//					vx = (float)(chassis_control_order.vy_set*sin(theta) + chassis_control_order.vx_set*cos(theta));    \
//					vy = (float)(chassis_control_order.vy_set*cos(theta) - chassis_control_order.vx_set*sin(theta));    \
//				}while(0)

			
typedef struct{
	
	float start_angle;			//电机初始角度值
	int start_angle_flag;	//记录电机初始角度值的flag
	int switch_mode_flag;  //记录模式转换角度值的flag
	int stop_angle;				//发送停止命令时候的角度值
	float target_angle;
	
	float actual_angle;			//当前真实角度值
	float last_angle;				//上一次返回的角度值
	float switch_mode_angle;  //记录模式转换角度值
	int round_cnt;				//相对开机时转过的圈数
	int total_angle;			//总共转过的计数
	
	float actual_speed;			//电机真实速度,rpm
	int target_speed;			//电机目标速度,rpm  转/min
	int last_speed;       //电机上一次回传的速度值
	int actual_current;		//电机真实电流
	int target_current;		//电机目标电流
	//int temp;							//电机温度（2006电机不支持，3508支持）
	float yaw_angle;
	float yaw_speed;
	float pitch_angle;
	float pitch_speed;
	PID_t pid;
	uint8_t spin_dirt;
}MOTOR_t;

typedef struct
{
	float vx_set;
	float vy_set;
	float wz_set;
	uint32_t chassis_mode;
	uint32_t last_chassis_mode;
	uint16_t gimbal_6020_angle;
	uint16_t last_gimbal_6020_angle;
}CHASSIS_CONTROL_ORDER_t;

typedef struct
{
	float real_vx;
	float real_vy;
}REAl_CHASSIS_SPEED_t;

typedef enum
{
	NO_STEP=0,
	X_STEP,
	Y_STEP,	
	XY_STEP,
}STEPSTAR;


extern MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
extern POWER_PID_t p_pid;
extern BUFFER_PID_t b_pid;
extern uint8_t fly_flag;
void chassis_move(void);
void limit_death(void);//模拟裁判系统的底盘功率限制扣血机制
#endif

