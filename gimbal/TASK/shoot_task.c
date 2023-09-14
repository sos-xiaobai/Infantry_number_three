#include "shoot_task.h"
#include "remote_control.h"
#include "sent_task.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "bsp_math.h"
#include <math.h>
#include "rc_task.h"
uint8_t data[2]={1,0};
shoot_task_t rc_shoot;
FRIC_SPEED fricspeed=FRIC_MIN;
extern float Ren;
int shoot_times=0;
int One_Shoot_flag=0;
int Ten_Shoot_flag=0;
int16_t SHOOT_LEFT_FRIC_SPEED_MAX=-7100;//30
int16_t SHOOT_LEFT_FRIC_SPEED_MIN=-7100;//15  4600
int16_t SHOOT_RIGHT_FRIC_SPEED_MAX=7100;//30
int16_t SHOOT_RIGHT_FRIC_SPEED_MIN=7100;//15 4600
extern float shoot_speed;
extern uint8_t speed_limit;
float TriggerSpeed[5] 	= {4.5f,		0.1f,		0.0f,		8000,			0};  //拨弹轮速度环 8000 4.8

float TriggerAngle[5] 	= {0.25f,		0.0000f,	 0.00f,		3000,			0};  //拨弹轮角度环 1700

float FricLeftSpeed[5] 	= {18.0f,         9.0,      0.0f,    10000.0f,      5000.0f};  //左摩擦轮速度环

float FricRightSpeed[5] = {18.0f,         6.0f,      0.0f,    10000.0f,      5000.0f};  //右摩擦轮速度环

static float abs_f(float a);

void trigger_angle_set(void);
//更新发子弹个数
static void shoot_angle_clc(void);
//防卡弹
//static void prevent_jam(void);
//摩擦轮PID计算
static void fric_pid(void);
//波弹轮PID计算
static void trigger_pid(void);
//摩擦轮射速控制
void fric_speed_control(void);

int trigger_back_flag,trigger_back_flag_cnt=0;
float speed_1;
float speed_2;
//发射任务
void shoot_task(void)
{
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //关掉激光
	if(trigger_cnt_flag==1)
		trigger_cnt++;
	
fric_speed_control();
	fric_pid();
	
	if((rc_shoot.left_fric.actual_speed<-4000)&&(rc_shoot.right_fric.actual_speed>4000)) //等待摩擦轮转动之后才能拨弹
	  shoot_angle_clc();

	trigger_pid();

	canTX_fric(rc_shoot.left_fric.set_currunt,rc_shoot.right_fric.set_currunt,rc_shoot.trigger.set_currunt);
}

float fric_min,fric_max=1;
float a,b=50;
//void fric_speed_control(void)
//{
//  if(shoot_speed!=last_shoot_speed)
//  {
//    if(fricspeed==FRIC_MIN) //15
//		{
//			   if(speed_limit-shoot_speed<1.0f) 
//			    {
//						SHOOT_LEFT_FRIC_SPEED_MIN+=50;
//						SHOOT_RIGHT_FRIC_SPEED_MIN-=50;
//			   }
//			   else if(speed_limit-shoot_speed>1.0f)
//			    { 
//						SHOOT_LEFT_FRIC_SPEED_MIN-=50;
//						SHOOT_RIGHT_FRIC_SPEED_MIN+=50;						
//			    }
//		}
//		else if(fricspeed==FRIC_MAX)  //30
//		{
//			   if(speed_limit-shoot_speed<1.0f) 
//			    {
//						SHOOT_LEFT_FRIC_SPEED_MAX+=50;
//						SHOOT_RIGHT_FRIC_SPEED_MAX-=50;
//			    }
//			   else if(speed_limit-shoot_speed>1.0f)
//			    { 
//						SHOOT_LEFT_FRIC_SPEED_MAX-=50;
//						SHOOT_RIGHT_FRIC_SPEED_MAX+=50;
//			    }
//		}
//  }
//	//读取到裁判系统射速限制  还有更多限制阶段超级对抗赛再说
//	if(speed_limit==30) fricspeed=FRIC_MAX; 
//	else fricspeed=FRIC_MIN;
//  last_shoot_speed=shoot_speed;
//}

float speed_average;
int yuyuyu_flag=65;
uint16_t count;
void fric_speed_control(void)
{
//	if(speed_limit==30) fricspeed=FRIC_MAX; 
	 fricspeed=FRIC_MIN;

}

//拨弹轮和摩擦轮的初始化
void shoot_init(void)
{
	
	PID_Init(&rc_shoot.trigger.speed_pid,TriggerSpeed);
	PID_Init(&rc_shoot.trigger.angle_pid,TriggerAngle);
	
    PID_Init(&rc_shoot.left_fric.speed_pid,FricLeftSpeed);
	PID_Init(&rc_shoot.right_fric.speed_pid,FricRightSpeed);
	
	rc_shoot.trigger.actual_angle = 0.0f;
	rc_shoot.trigger.actual_speed = 0.0f;
	
	rc_shoot.trigger.last_shoot_flag = 0;
	rc_shoot.trigger.last_back_flag = 0;
	rc_shoot.trigger.target_angle = 0.0f;
	rc_shoot.trigger.target_speed = 0.0f;
	
	rc_shoot.left_fric.actual_speed = 0.0f;
	rc_shoot.left_fric.target_speed = 0;
	
	rc_shoot.right_fric.actual_speed = 0.0f;
	rc_shoot.right_fric.target_speed = 0;

}

static float abs_f(float a)
 {
	if(a<0)a=-1.0f*a;
	return a;
 }

void check_fric(void)
{
	if(abs(rc_shoot.left_fric.actual_speed-rc_shoot.left_fric.target_speed)>750) shoot_close_flag=1;
	else if(abs(rc_shoot.right_fric.actual_speed-rc_shoot.right_fric.target_speed)>750) shoot_close_flag=1;
	else shoot_close_flag=0;
}
 
int trigger_round;
int shoot_trigget_cnt=0;
static void shoot_angle_clc(void)
{
	check_fric();
	if(shoot_close_flag==1) return;
	
	if(rc_shoot.trigger.rounds<=-90)
	{
		rc_shoot.trigger.target_angle-=(float)(rc_shoot.trigger.rounds)*8191.0f;
		if(rc_shoot.trigger.target_angle>0) rc_shoot.trigger.target_angle=0;
		rc_shoot.trigger.total_angle -=(float)(rc_shoot.trigger.rounds)*8191.0f;
	  rc_shoot.trigger.begin_angle=rc_shoot.trigger.total_angle;
		rc_shoot.trigger.rounds=0;
	}

	if(rc_shoot.trigger.last_shoot_flag==0)
	{
		if(One_Shoot_flag==1||Ten_Shoot_flag==1)  
		{
//			if(shoot_trigger_cnt_flag==1) 
//			{
//				if(One_Shoot_flag==1)
//					shoot_trigget_cnt++;
//				else if(Ten_Shoot_flag==1)
//					shoot_trigget_cnt+=5;
//					if(shoot_trigget_cnt>44) 
//					{
//						shoot_trigget_cnt=0;
//						shoot_trigger_cnt_flag=0;
//						shoot_close_flag=1;
//					}
//     	}
			rc_shoot.trigger.last_shoot_flag=1;
			trigger_angle_set();
			shoot_times+=1;
			if(shoot_times>6) shoot_times=0;
		}
	}
		
	if(One_Shoot_flag!=1 && Ten_Shoot_flag!=1) rc_shoot.trigger.last_shoot_flag=0;

}
static void fric_pid(void)
{
	
	PID_Calc(&rc_shoot.left_fric.speed_pid,rc_shoot.left_fric.target_speed,rc_shoot.left_fric.actual_speed);
	
	rc_shoot.left_fric.set_currunt=rc_shoot.left_fric.speed_pid.out;
	
	PID_Calc(&rc_shoot.right_fric.speed_pid,rc_shoot.right_fric.target_speed,rc_shoot.right_fric.actual_speed);
	
	rc_shoot.right_fric.set_currunt=rc_shoot.right_fric.speed_pid.out;
}


static void trigger_pid(void)
{
//	rc_shoot.trigger.target_angle=Ren;    //用于测试速度环
	PID_Calc(&rc_shoot.trigger.angle_pid,rc_shoot.trigger.target_angle,rc_shoot.trigger.total_angle);
	
	rc_shoot.trigger.target_speed = rc_shoot.trigger.angle_pid.out;
//	rc_shoot.trigger.target_speed=Ren;    //用于测试角度环
	PID_Calc(&rc_shoot.trigger.speed_pid,rc_shoot.trigger.target_speed,rc_shoot.trigger.actual_speed);
	
	rc_shoot.trigger.set_currunt = rc_shoot.trigger.speed_pid.out;
}
int trigger_cnt=0,trigger_cnt_flag=0,remain_bullet=0;
uint8_t flag__b=0;
void trigger_angle_set(void)
{
	trigger_cnt_flag=1;
//	remain_bullet=((heat_limit-heat)/10)-2;  //计算热量限制下的剩余弹量
//	if(remain_bullet<0) remain_bullet=0;
	remain_bullet=100000000;
	if(One_Shoot_flag==1) //单发
	{
		if(remain_bullet>=1)
			if(fabs(rc_shoot.trigger.target_angle-rc_shoot.trigger.total_angle)<((1.0f*36.0f*(360.0f/8.0f))/360.0f*8191.0f))
		{
		  rc_shoot.trigger.target_angle-=((1.0f*36.0f*(360.0f/8.0f))/360.0f*8191.0f);
			flag__b++;
			if(flag__b>=127) flag__b=0;
		}
	}
	else //五连发
	{
		if(fabs(rc_shoot.trigger.target_angle-rc_shoot.trigger.total_angle)<((1.0f*36.0f*(360.0f/8.0f))/360.0f*8191.0f))
		{
			if(remain_bullet>=5)
		    rc_shoot.trigger.target_angle-=((5.0f*36.0f*(360.0f/8.0f))/360.0f*8191.0f);//改成五连发了
		  else
			  rc_shoot.trigger.target_angle-=((remain_bullet*36.0f*(360.0f/8.0f))/360.0f*8191.0f);
		}
	}
}
//拨弹轮数据返回
void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed,int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;
	motor->actual_current=current;

	if(motor->record_begin_angle_status==0)
	{
		motor->begin_angle = angle;
		motor->record_begin_angle_status++;
	}
	if(motor->actual_angle - motor->last_angle > 4896)
		motor->rounds --;
	else if (motor->actual_angle - motor->last_angle < -4896)
		motor->rounds ++;
	motor->total_angle = motor->rounds * 8192 + motor->actual_angle;
}

