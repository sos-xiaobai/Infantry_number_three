#include "gimbal_task.h"
#include "vision_task.h"
#include "gimbal_task_behaviour.h"
#include "sent_task.h"
#include "bsp_imu.h"
#include "can_receive.h"
#include "shoot_task.h"
#include <math.h>
#include "tim.h"

GIMBAL_t gimbal_y,gimbal_p;
GIMBAL_MODE_t gimbal_set_mode;
//yaw轴PID控制
float YawGyroPid[6] 	= {0.1f,			0.25f,			0.0,	3.6f,			0.0f,	 0.0f};  //imu角度环  1.8 
float YawEncondePid[6] 	= {0.3f,			0.0f,			0.0f,		50.0f,			0.0f,		0.0f};  //编码器角度环  rhn 1
float YawEncondePidSpeed[6] 	= {500.0f,			50.0f,			0.0f,		10000.0f,			3000.0f,		0.0f};  //编码器速度环  rhn 3
float YawSpeedPid[6] 			= {20000.0f,			500.0f,			2000.0f,		13000.0f,			6000.0f,		0.01f};     //速度环  rhn 7000  9000
//pitch轴PID控制
float PitchGyroPid[6] = {50.0f,			0.0f,			1.0f,		500.0f,			0.0f		,0.0f};
float PitchEncondePid[6] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
float PitchEncondePidSpeed[6] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
float PitchSpeedPid[6] 		= {18000.0f,			20.0f,			0.0f,		3000.0f,	1000.0f,	0.00f};
 

//float PitchGyroPid[6] = {0.12f,			0.0f,			0.0f,		70.0f,			0.0f		,0.0f};
//float PitchEncondePid[6] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
//float PitchEncondePidSpeed[6] = {0.0f,			0.0f,			0.0f,		0.0f,			0.0f	,0.0f};
//float PitchSpeedPid[6] 		= {70.0f,			2.0f,			0.0f,		500.0f,	50.0f,	0.00f};


//PID初始化	
static void YawPitch_PIDinit(void);
//云台模式选择
static void GIMBAL_Set_Mode(void);
static void gimbal_rc_mode(void);
//云台控制
static void GIMBAL_Set_Contorl(void);
//PID计算
static void GIMBAL_PID(void);
//IMU数据接收
static void GIMBAL_CALBACK_GET(void);
//速度PID
static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_);
static void gimbal_motor_encode_pid_y(GIMBAL_t *gimbal_);
int gimbal_imu_cnt=0;
uint16_t set_compare=1200;
void Gimbal_Task(void)
{
	//舵机
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,set_compare);
	//延时等待imu数据稳定
	//IMU 数据接收
	GIMBAL_CALBACK_GET();
	//模式选择
	GIMBAL_Set_Mode();
	//模式控制
	GIMBAL_Set_Contorl();
	//PID计算	
	GIMBAL_PID();

	canTX_gimbal_p(gimbal_p.gimbal_gyro_pid.out);
//	canTX_gimbal_p_2(gimbal_p.gimbal_raw_pid.out);
	
}
//云台初始化
void Gimbal_Init(void)
{
	YawPitch_PIDinit(); //PID初始化
	gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
	//yaw轴数据初始化
	gimbal_y.IMU_actual_angle=0.0f;
	gimbal_y.IMU_actual_speed=0.0f;
	gimbal_y.CAN_actual_angle=0.0f;
	gimbal_y.CAN_actual_speed=0.0f;
	gimbal_y.target_angle=0.0f;
	gimbal_y.target_speed=0.0f;
	gimbal_y.add_angle=0.0f;
	gimbal_y.given_current=0;
	
	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_RAW   GIMBAL_MOTOR_GYRO;//yaw轴电机模式控制
	
	//pitch轴数据控制
	gimbal_p.IMU_actual_angle=0.0f;
	gimbal_p.IMU_actual_speed=0.0f;
	gimbal_p.CAN_actual_angle=0.0f;
	gimbal_p.CAN_actual_speed=0.0f;
	gimbal_p.target_angle=0.0f;
	gimbal_p.target_speed=0.0f;
	gimbal_p.add_angle=0.0f;
	gimbal_p.given_current=0;
	
	gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//GIMBAL_MOTOR_RAW   GIMBAL_MOTOR_GYRO;//pitch轴电机控制
}

static void YawPitch_PIDinit(void)
{
	//yaw轴PID数据初始化
	PID_Init(&gimbal_y.gimbal_raw_pid,YawSpeedPid);//陀螺仪速度控制
	PID_Init(&gimbal_y.gimbal_gyro_pid,YawGyroPid);//陀螺仪角度控制
	PID_Init(&gimbal_y.gimbal_enconde_pid,YawEncondePid);//编码器角度控制
	PID_Init(&gimbal_y.gimbal_enconde_pid_speed,YawEncondePidSpeed);//编码器速度控制

	//pitch轴PID数据初始化
	PID_Init(&gimbal_p.gimbal_raw_pid,PitchSpeedPid);//陀螺仪速度控制
	PID_Init(&gimbal_p.gimbal_gyro_pid,PitchGyroPid);//陀螺仪角度控制
	PID_Init(&gimbal_p.gimbal_enconde_pid,PitchEncondePid);//编码器角度控制
	PID_Init(&gimbal_p.gimbal_enconde_pid_speed,PitchEncondePidSpeed);	//编码器速度控制
}


static void GIMBAL_Set_Mode(void)
{
	
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_rc_mode();
	//电机模式选择
	if(gimbal_set_mode==GIMBAL_INIT)//初始化模式
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;//陀螺仪角度控制
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_ZERO_FORCE)//无力状态
	{
	  gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;//陀螺仪速度控制
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
	}
	else if(gimbal_set_mode==GIMBAL_ABSOLUTE_ANGLE)//云台随动模式
	{
//		YawGyroPid[5]=0.2;  //小陀螺模式死区加大
//		PitchGyroPid[5]=0.4;
		gimbal_y.gimbal_gyro_pid.deadband=0.0f;
		gimbal_p.gimbal_gyro_pid.deadband=0.0f;
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_TOP_ANGLE)//小陀螺
	{
//		YawGyroPid[5]=0.5;  //小陀螺模式死区加大
//		PitchGyroPid[5]=0.8;
//		gimbal_y.gimbal_gyro_pid.deadband=0.2f;
//		gimbal_p.gimbal_gyro_pid.deadband=0.7f;
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_RELATIVE_ANGLE)//编码器角度控制
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
	}
	else if(gimbal_set_mode==GIMBAL_CALI)//电机速度控制模式
	{
		gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
		gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
	}
//	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_ENCONDE;
	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_GYRO;
//	gimbal_y.gimbal_motor_mode=GIMBAL_MOTOR_RAW;
}

static void gimbal_rc_mode(void)
{
	if(gimbal_set_mode==GIMBAL_INIT)
	{
		static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
		
		if((fabs(gimbal_y.IMU_actual_angle-INIT_YAW_SET)<GIMBAL_INIT_ANGLE_ERROR)&&(fabs(gimbal_p.IMU_actual_angle-INIT_PITCH_SET)<GIMBAL_INIT_ANGLE_ERROR))
		{
			 //到达初始化位置
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
		}
		//上电无法定位在中间位置
		if(init_time>GIMBAL_INIT_TIME&&init_stop_time < GIMBAL_INIT_STOP_TIME)
		{
			gimbal_set_mode=GIMBAL_ZERO_FORCE;
		}
		if(init_stop_time > GIMBAL_INIT_STOP_TIME)
		{
			gimbal_set_mode= GIMBAL_ABSOLUTE_ANGLE; //云台随动
		}
	}
//	else gimbal_set_mode=GIMBAL_ZERO_FORCE; //调试用
}

static void GIMBAL_Set_Contorl(void)
{
	//选定期望值来源
	Vision_Task();
	
	//模式选择
   gimbal_control_behaviour();
}
double gimbalp_imuangle;
uint8_t first_update_flag=0;
float imu_can_error_y=0.0f;
static void GIMBAL_CALBACK_GET(void)
{
	//yaw轴数据返回
    gimbal_y.CAN_actual_angle=yaw_can_rx.angle; //计算yaw的真实角度
	gimbal_y.CAN_total_angle =yaw_can_rx.turns*360.0f+gimbal_y.CAN_actual_angle;		//计算yaw的total_angle
	gimbal_y.CAN_actual_speed=yaw_can_rx.speed;
	//yaw轴IMU数据获取
    gimbal_y.IMU_actual_angle=INS_angle[0]/(2*3.141590f)*360.0f;
	gimbal_y.IMU_actual_speed=INS_gyro[2]*0.5f;
	
	//pitch数据返回
	gimbal_p.CAN_actual_angle=pitch_can_rx.angle/8191.0f*360.0f;
	gimbal_p.CAN_actual_speed=pitch_can_rx.speed;
	//pitchIMU数据获取
    gimbal_p.IMU_actual_angle=-1.00f*INS_angle[2]/(2*3.141590f)*360.0f;
	gimbal_p.IMU_actual_speed=1.00f*INS_gyro[1];
	
	
	if(first_update_flag==0)  //读取上电时yaw编码器角度和imu角度的固定差值  并且只读取一次
	{
		imu_can_error_y=gimbal_y.IMU_actual_angle-(gimbal_y.CAN_actual_angle*360.0f/8192.0f);
	}
	first_update_flag=1;
}

//云台PId任务
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
		if((gimbal_y.target_angle-gimbal_y.CAN_actual_angle)>4096)
			gimbal_y.target_angle -= 8192;
		if((gimbal_y.target_angle-gimbal_y.CAN_actual_angle)<-4096)
			gimbal_y.target_angle += 8192;
		
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
//	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->CAN_actual_speed);	

	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}

static void gimbal_motor_gyro_pid(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_gyro_pid,gimbal_->target_angle,gimbal_->IMU_actual_angle);
	
	gimbal_->target_speed=gimbal_->gimbal_gyro_pid.out;
	
//	if(gimbal_->gimbal_gyro_pid.out>2.0f) gimbal_->gimbal_gyro_pid.out=2.0f; 
//	if(那个值!=0)gimbal_->target_speed+=那个值;
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}

float temp_w;
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_)
{	
	//gimbal_->target_angle=temp_w;
	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_actual_angle);     //pitch的第三个参数为gimbal_->CAN_actual_angle

//	gimbal_->target_speed=temp_w;//gimbal_->gimbal_enconde_pid.out
	
	gimbal_->target_speed=gimbal_->gimbal_enconde_pid.out;
	
//    PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
//	
//	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_raw_pid.out;
}
static void gimbal_motor_encode_pid_y(GIMBAL_t *gimbal_)
{	
	//gimbal_->target_angle=temp_w;
//	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_total_angle);     //pitch的第三个参数为gimbal_->CAN_actual_angle
	PID_Calc(&gimbal_->gimbal_enconde_pid,gimbal_->target_angle,gimbal_->CAN_actual_angle);
//	gimbal_->target_speed=temp_w;//gimbal_->gimbal_enconde_pid.out
	gimbal_->target_speed=gimbal_->gimbal_enconde_pid.out;	
	//		gimbal_->target_speed=Ren;	
//    PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
//	
//	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;
//	PID_Calc(&gimbal_->gimbal_raw_pid,gimbal_->target_speed,gimbal_->IMU_actual_speed);
	PID_Calc(&gimbal_->gimbal_enconde_pid_speed,gimbal_->target_speed,gimbal_->CAN_actual_speed);
	
	gimbal_->given_current=gimbal_->gimbal_enconde_pid_speed.out;


}

