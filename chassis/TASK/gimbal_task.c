#include "gimbal_task.h"
#include "vision_task.h"
#include "gimbal_task_behaviour.h"
#include "sent_task.h"
#include "bsp_imu.h"
#include "can_receive.h"
#include "shoot_task.h"
#include <math.h>
GIMBAL_t gimbal_y,gimbal_p;
// 42-0
GIMBAL_MODE_t gimbal_set_mode;

float YawGyroPid[6] 	= {0.12f,			0.0f,			0.0f,	4.0f,			0.0f,	0.0f};  //imu角度环
float YawEncondePid[6] 	= {1.2f,			0.0f,			0.2f,		14.0f,			0.0f,		0.0f};  //编码器角度环
float YawEncondePidSpeed[6] 	= {1500.0f,			500.0f,			0.0f,		20000.0f,			5000.0f,		0.0f};  //编码器角度环
float YawSpeedPid[6] 			= {11200.0f,			1400.0f,			0.0f,		29500.0f,			9800.0f,		0.05f};     //速度环



float PitchGyroPid[6] = {0.23f,			0.0f,			0.0f,		4.5f,			0.0f		,0.0f};
float PitchEncondePid[6] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
float PitchEncondePidSpeed[6] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
float PitchSpeedPid[6] 		= {6300.0f,			300.0f,			0.0f,		22000.0f,	1000.0f,	0.0f};

//float PitchGyroPid[5] = {0.63f,			0.000f,			0.0f,		5.0f,			1.4f};
//float PitchEncondePid[5] = {1.6f,			0.0f,			1.0f,		10.0f,			0.0f};
//float PitchEncondePidSpeed[5] = {1300.0f,			600.0f,			0.0f,		15000.0f,			12000.0f};
//float PitchSpeedPid[5] 		= {4000.0f,			16.6f,			0.0f,		16500.0f,			2500.0f};


//PID初始化	
static void YawPitch_PIDinit(void);
//云台模式选择
static void GIMBAL_Set_Mode(void);

//云台控制
static void GIMBAL_Set_Contorl(void);
//PID计算
static void GIMBAL_PID(void);
//IMU数据接收
static void GIMBAL_CALBACK_GET(void);

static void gimbal_prtrol_set(void);

//速度PID
static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_encode_pid_y(GIMBAL_t *gimbal_);
int gimbal_imu_cnt=0;
int hhhh=0;
int yuu=0;
float xuan=0;
void Gimbal_Task(void)
{
	//延时等待imu数据稳定
	//IMU 数据接收
	GIMBAL_CALBACK_GET();
	//模式选择
	GIMBAL_Set_Mode();
	//模式控制
	GIMBAL_Set_Contorl();
	//PID计算	
	GIMBAL_PID();
	//设定电流值
	gimbal_p.given_current+=-4300; //用于平衡pitch负载

//	if(gimbal_p.IMU_actual_angle<-25 || gimbal_p.IMU_actual_angle>35)
//		yuu=1;
//	if(yuu==1)
//	gimbal_p.given_current=0;
//	if(gimbal_y.IMU_actual_angle<-130 || gimbal_y.IMU_actual_angle>130)
//		yuu=1;
//	else yuu=0;
//	if(yuu==1)
//	gimbal_y.given_current=0;

	canTX_gimbal(gimbal_p.given_current,gimbal_y.given_current);
	
}
//云台初始化
void Gimbal_Init(void)
{
	
	YawPitch_PIDinit(); //PID初始化
	gimbal_set_mode=VISION_OFF_ALL;
	gimbal_y.IMU_actual_angle=0.0f;
	gimbal_y.IMU_actual_speed=0.0f;
	gimbal_y.CAN_actual_angle=0.0f;
	gimbal_y.CAN_actual_speed=0.0f;
	gimbal_y.target_angle=0.0f;
	gimbal_y.target_speed=0.0f;
	gimbal_y.add_angle=0.0f;
	gimbal_y.given_current=0;
	
	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_RAW   GIMBAL_MOTOR_GYRO;
	
	gimbal_p.IMU_actual_angle=0.0f;
	gimbal_p.IMU_actual_speed=0.0f;
	gimbal_p.CAN_actual_angle=0.0f;
	gimbal_p.CAN_actual_speed=0.0f;
	gimbal_p.target_angle=20.0f;
	gimbal_p.target_speed=0.0f;
	gimbal_p.add_angle=0.0f;
	gimbal_p.given_current=0;
	
	gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_RAW   GIMBAL_MOTOR_GYRO;
}

static void YawPitch_PIDinit(void)
{
	PID_Init(&gimbal_y.gimbal_raw_pid,YawSpeedPid);
	PID_Init(&gimbal_y.gimbal_gyro_pid,YawGyroPid);
	PID_Init(&gimbal_y.gimbal_enconde_pid,YawEncondePid);
	PID_Init(&gimbal_y.gimbal_enconde_pid_speed,YawEncondePidSpeed);
	
	PID_Init(&gimbal_p.gimbal_raw_pid,PitchSpeedPid);
	PID_Init(&gimbal_p.gimbal_gyro_pid,PitchGyroPid);
	PID_Init(&gimbal_p.gimbal_enconde_pid,PitchEncondePid);
	PID_Init(&gimbal_p.gimbal_enconde_pid_speed,PitchEncondePidSpeed);	
}

static void GIMBAL_Set_Mode(void)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
	//电机模式选择
	if(gimbal_set_mode==VISION_ON_ALL)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==VOLUNTARIY_PATROL)
	{
	  gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==VISION_OFF_ALL)
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	Protrol_Judge();
}


static void GIMBAL_Set_Contorl(void)
{

	if(gimbal_prtrol==PATROL_ON)
	{
		gimbal_prtrol_set();
	}
	else
	{
		//选定期望值来源
	    Vision_Task();
	}
		
	gimbal_control_behaviour();
	
}
double gimbalp_imuangle;
static void GIMBAL_CALBACK_GET(void)
{
  gimbal_y.CAN_actual_angle=yaw_can_rx.angle/8191.0f*360.0f;
	gimbal_y.CAN_total_angle=yaw_can_rx.turns*360.0f+gimbal_y.CAN_actual_angle;		//计算yaw的total_angle
	gimbal_y.CAN_actual_speed=yaw_can_rx.speed;
	
  gimbal_y.IMU_actual_angle=-INS_angle[0]/(2*3.141590f)*360.0f;
	gimbal_y.IMU_actual_speed=-INS_gyro[2]*0.5f;
	
	
	gimbal_p.CAN_actual_angle=pitch_can_rx.angle/8191.0f*360.0f;
	gimbal_p.CAN_actual_speed=pitch_can_rx.speed;
	
  gimbal_p.IMU_actual_angle=1.00f*INS_angle[2]/(2*3.141590f)*360.0f;
	gimbal_p.IMU_actual_speed=-1.00f*INS_gyro[1];
}



static void GIMBAL_PID(void)
{
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_pid(&gimbal_y);
	}
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		if(gimbal_y.target_angle-gimbal_y.IMU_actual_angle>180)
			gimbal_y.target_angle -= 360;
		if(gimbal_y.target_angle-gimbal_y.IMU_actual_angle<-180)
			gimbal_y.target_angle += 360;	 
		
		gimbal_motor_gyro_pid(&gimbal_y);
	}
	if(gimbal_y.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_encode_pid_y(&gimbal_y);
	}
	
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_RAW)
	{
		gimbal_motor_raw_pid(&gimbal_p);
	}
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_GYRO)
	{
		gimbal_motor_gyro_pid(&gimbal_p);
	}
	if(gimbal_p.gimbal_motor_mode==GIMBAL_MOTOR_ENCONDE)
	{
		gimbal_motor_encode_pid(&gimbal_p);
	}
}

static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}

static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_gyro_pid,gimbal_->target_angle,gimbal_->IMU_actual_angle);
	
	gimbal_->target_speed=gimbal_->gimbal_gyro_pid.out;
	
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
//	if(gimbal_y.gimbal_gyro_pid.error[0]<=0.3f || gimbal_y.gimbal_gyro_pid.error[0]>=-0.3f)
//	{
//		if(gimbal_y.gimbal_raw_pid.out>400)
//			gimbal_y.gimbal_raw_pid.out=400;
//		if(gimbal_y.gimbal_raw_pid.out<-400)
//			gimbal_y.gimbal_raw_pid.out=-400;
//	}
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}

float temp_w;
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_)
{	
	//gimbal_->target_angle=temp_w;
	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_actual_angle);     //pitch的第三个参数为gimbal_->CAN_actual_angle

//	gimbal_->target_speed=temp_w;//gimbal_->gimbal_enconde_pid.out
	
	gimbal_->target_speed=gimbal_->gimbal_enconde_pid.out;
	
    PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
}
static void gimbal_motor_encode_pid_y(GIMBAL_t *gimbal_)
{	
	//gimbal_->target_angle=temp_w;
	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_total_angle);     //pitch的第三个参数为gimbal_->CAN_actual_angle

//	gimbal_->target_speed=temp_w;//gimbal_->gimbal_enconde_pid.out
	
	gimbal_->target_speed=gimbal_->gimbal_enconde_pid.out;
	
    PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
}

static void gimbal_prtrol_set(void)
{
	gimbal_y.add_angle=PRTROL_ANGKLE_YAW;
	gimbal_p.add_angle=PRTROL_ANGKLE_PITCH;
}
