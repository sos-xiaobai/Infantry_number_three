#include "pid.h"
#include "fuzzy_pid.h"
#include "chassis_move.h"

switch_flag_t switch_flag;


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//角度环实现
static void apid_realize(APID_t *apid,Parameter_t *loop_p);
//速度环实现
static void vpid_realize(VPID_t *vpid,Parameter_t *loop_p);

static void position_loop_realize(PID_Loop_t *loop_select);
static void speed_loop_realize(PID_Loop_t *loop_select);

static void position_loop_init(PID_Loop_t *loop_select);
static void speed_loop_init(PID_Loop_t *loop_select);
static void APID_Init(APID_t *apid);
static void VPID_Init(VPID_t *vpid);

int CHASSIS_vPID_max=25000;
int CHASSIS_Integral_max=5000;
void motor_pid_init(PID_t *pid)
{
	if(pid->loop_flag==POSITION_LOOP)
		position_loop_init(&(pid->position_loop));
	else if(pid->loop_flag==SPEED_LOOP)
	speed_loop_init(&(pid->speed_loop));
}

void POWER_PID_Init(POWER_PID_t *pid)
{
	pid->actual_power=0;
	pid->target_power=0;
	pid->err=0;
	pid->last_err=0;
	pid->err_integration=0;
	pid->P_OUT=0;
	pid->I_OUT=0;
	pid->D_OUT=0;
	pid->PID_OUT=0;
}

void BUFFER_PID_Init(BUFFER_PID_t *pid)
{
	pid->actual_buffer=0;
	pid->target_buffer=0;
	pid->err=0;
	pid->err_max=9;
	pid->last_err=0;
	pid->err_integration=0;
	pid->err_integration_max=120;
	pid->P_OUT=0;
	pid->I_OUT=0;
	pid->D_OUT=0;
	pid->PID_OUT=0;
}
void power_pid_realize(POWER_PID_t *pid)
{
	pid->err=pid->target_power-pid->actual_power;
	if(pid->err <= POWER_IntegralSeparation&& pid->err >= -POWER_IntegralSeparation)		//积分分离
	pid->err_integration += pid->err;
	if(pid->err_integration > POWER_Integral_max)		//抗积分饱和
	pid->err_integration = POWER_Integral_max;
	else if(pid->err_integration < -POWER_Integral_max)
	pid->err_integration = -POWER_Integral_max;
			
  pid->P_OUT = pid->kp * pid->err;
	pid->I_OUT = pid->ki * pid->err_integration;		//I项
	pid->D_OUT = pid->kd * (pid->err-pid->last_err);
	pid->last_err = pid->err;
	
	if((pid->P_OUT + pid->D_OUT+pid->I_OUT) > PID_OUT_MAX_POWER) 			
	pid->PID_OUT = P_OUT_MAX_POWER;
	else if((pid->P_OUT + pid->D_OUT+pid->I_OUT) < -PID_OUT_MAX_POWER) 
	pid->PID_OUT = -P_OUT_MAX_POWER;
	else
	pid->PID_OUT = pid->P_OUT +	pid->D_OUT+pid->I_OUT;
}

void buffer_pid_realize(BUFFER_PID_t *pid)
{
	pid->err=pid->target_buffer-pid->actual_buffer;
			
    
	if(pid->err>-pid->err_max&& pid->err<pid->err_max)
		pid->err_integration+=pid->err;
	
	if(pid->err_integration>pid->err_integration_max) pid->err_integration=pid->err_integration_max;
	if(pid->err_integration<-pid->err_integration_max)pid->err_integration=-pid->err_integration_max;
	
	pid->P_OUT = pid->kp * pid->err;
	pid->I_OUT = pid->ki * pid->err_integration;		//I项
	pid->D_OUT = pid->kd * (pid->err-pid->last_err);
	pid->last_err = pid->err;
		
	
	if((pid->P_OUT + pid->D_OUT+pid->I_OUT) > P_OUT_MAX_BUFFER) 			
	pid->PID_OUT = P_OUT_MAX_BUFFER;
	else if((pid->P_OUT + pid->D_OUT+pid->I_OUT) < -P_OUT_MAX_BUFFER) 
	pid->PID_OUT = -P_OUT_MAX_BUFFER;
	else
	pid->PID_OUT = pid->P_OUT +	pid->D_OUT+pid->I_OUT;
}

void pid_realize(PID_t *pid)
{
	if(pid->loop_flag==POSITION_LOOP)
		position_loop_realize(&(pid->position_loop));
	else if(pid->loop_flag==SPEED_LOOP)
	speed_loop_realize(&(pid->speed_loop));
}
static void position_loop_init(PID_Loop_t *loop_select)
{
	APID_Init(&(loop_select->apid));
	VPID_Init(&(loop_select->vpid));
}

static void speed_loop_init(PID_Loop_t *loop_select)
{
	VPID_Init(&(loop_select->vpid));
}

static void APID_Init(APID_t *apid)
{
	apid->actual_angle=0;
	apid->target_angle=0;
	apid->err=0;
	apid->last_err=0;
	apid->err_integration=0;
	apid->P_OUT=0;
	apid->I_OUT=0;
	apid->D_OUT=0;
	apid->PID_OUT=0;
}

static void VPID_Init(VPID_t *vpid)
{
	vpid->target_speed=0;
	vpid->actual_speed=0;
	vpid->err=0;
	vpid->last_err=0;
	vpid->err_integration=0;
	vpid->P_OUT=0;
	vpid->I_OUT=0;
	vpid->D_OUT=0;
	vpid->PID_OUT=0;
	vpid->pid_count=0;
	vpid->average_err=0;
	vpid->last_average_err=0; 
}

static void position_loop_realize(PID_Loop_t *loop_select)
{
	apid_realize(&(loop_select->apid),&(loop_select->pid_Parameter));
	loop_select->vpid.target_speed=loop_select->apid.PID_OUT;
	
	vpid_realize(&(loop_select->vpid),&(loop_select->pid_Parameter));
}


static void speed_loop_realize(PID_Loop_t *loop_select)
{
	vpid_realize(&(loop_select->vpid),&(loop_select->pid_Parameter));
}


static void apid_realize(APID_t *apid,Parameter_t *loop_p)
{
	apid->err = apid->target_angle - apid->actual_angle;
	switch(switch_flag)
	{
	case(CHASSIS):
		{
			apid->P_OUT = loop_p->Akp * apid->err;
	    apid->D_OUT = loop_p->Akd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX_CHASSIS;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX_CHASSIS;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
			}
		break;
	
	case(FOLLOW):
		{
			if(((apid->err<1.1f)&&(apid->err>-1.1f)))
				apid->err=0;
//			if(((apid->err<8)||(apid->err>-8))&&(chassis_center.switch_mode_flag==SPIN_TO_FOLLOW))
//				chassis_center.switch_mode_flag=MODE_NO_SWITCH;
//			
//			if(chassis_center.switch_mode_flag==SPIN_TO_FOLLOW)
//			{
//				if(chassis_center.spin_dirt==CLOCKWISE)
//				{
//					if(apid->err>0)
//						apid->err-=360.0f;
//				}
//				else if(chassis_center.spin_dirt==ANTICLOCKWISE)
//				{
//					if(apid->err<0)
//						apid->err+=360.0f;
//				}
//			}
			
			if(chassis_center.switch_mode_flag==MODE_NO_SWITCH)
			{
				if(apid->err>180.0f) apid->err-=360.0f;
				else if(apid->err<-180.0f) apid->err+=360.0f;
			}
			if(apid->err <= FOLLOW_aIntegralSeparation&& apid->err >= -FOLLOW_aIntegralSeparation)		//积分分离
			apid->err_integration += apid->err;
	    if(apid->err_integration > FOLLOW_Integral_amax)		//抗积分饱和
		  apid->err_integration = FOLLOW_Integral_amax;
	    else if(apid->err_integration < -FOLLOW_Integral_amax)
		  apid->err_integration = -FOLLOW_Integral_amax;
			
    	apid->P_OUT = loop_p->Akp * apid->err;
			apid->I_OUT = loop_p->Aki * apid->err_integration;		//I项
	    apid->D_OUT = loop_p->Akd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
		
		  if((apid->P_OUT ) > aP_OUT_MAX_FOLLOW) 			
	   	apid->P_OUT = aP_OUT_MAX_FOLLOW;
			if((apid->P_OUT) < -aP_OUT_MAX_FOLLOW) 			
	   	apid->P_OUT = -aP_OUT_MAX_FOLLOW;
	
			if((apid->P_OUT + apid->D_OUT+apid->I_OUT) > aPID_OUT_MAX_FOLLOW) 			
	   	apid->PID_OUT = aPID_OUT_MAX_FOLLOW;
	    else if((apid->P_OUT + apid->D_OUT+apid->I_OUT) < -aPID_OUT_MAX_FOLLOW) 
		  apid->PID_OUT = -aPID_OUT_MAX_FOLLOW;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT+apid->I_OUT;
    }
		break;
		default:break;
	}
}

//CHASSIS_SPIN
static void vpid_realize(VPID_t *vpid,Parameter_t *loop_p)
{
	vpid->err = vpid->target_speed - vpid->actual_speed;
	switch(switch_flag)
	{
	case(CHASSIS):
	 {
		if(abs(vpid->err) <= CHASSIS_IntegralSeparation)		//积分分离
			vpid->err_integration += vpid->err;
	  if(vpid->err_integration > CHASSIS_Integral_max)		//抗积分饱和
		vpid->err_integration = CHASSIS_Integral_max;
	  else if(vpid->err_integration < -CHASSIS_Integral_max)
		vpid->err_integration = -CHASSIS_Integral_max;
		
		vpid->P_OUT = loop_p->Vkp * vpid->err;								//P项
	  vpid->I_OUT = loop_p->Vki * vpid->err_integration;		//I项
		
		//输出限幅
		if((vpid->P_OUT + vpid->I_OUT )> CHASSIS_vPID_max) 
		vpid->PID_OUT = CHASSIS_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -CHASSIS_vPID_max) 
		vpid->PID_OUT = -CHASSIS_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
   }
	 break;
	
	default:break;
	}
}

		
		
//pid计算

float PID_Calc(PidTypeDef *pid,  float ref,float fdb)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->ref = ref;
    pid->fdb = fdb;
    pid->error[0] = ref - fdb;
		
		if(pid->error[0]> -pid->deadband && pid->error[0]<pid->deadband) //死区
			pid->error[0]=0;

 
	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dbuf[2] = pid->Dbuf[1];
	pid->Dbuf[1] = pid->Dbuf[0];
	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Dbuf[0];
	LimitMax(pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	LimitMax(pid->out, pid->max_out);
//		if(pid->error[0]>-pid->deadband&&pid->deadband)//死区执行积分分离操作
//			pid->out=pid->Pout+pid->Dout;
//			else
//				pid->out=pid->Pout+pid->Iout+pid->Dout;
//		
    return pid->out;
}

//pid参数初始化
void PID_Init(PidTypeDef *pid,  const float PID[6])
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = PID[3];
    pid->max_iout = PID[4]; 
		pid->deadband= PID[5];
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

//pid复位
void PID_clear(PidTypeDef *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->ref = 0.0f;
}
