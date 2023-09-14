//#include "lqr.h"
//#include "fuzzy_pid.h"
//#include <math.h>
//#include "imu.h"
//#include "bsp_math.h"
//#include "arm_math.h"
//#include "chassis_move.h"
//#include "line_fit.h"
////#include "matrix.h"
////#include "state.h"
//float PID_LQR_x[6] 			 = {100.0f,		0.0f,			0.0f,		150.0f,			0.0f,		0.0f};     
//float PID_LQR_x_v[6] 		 = {70.0f,		0.5f,		  0.0f,	  300.0f,		  25.0f,	0.0f};
//float PID_LQR_pitch[6] 	 = {10.0f,		0.0f,		0.0f,	  400.0f,			0.0f,		0.0f};
//float PID_LQR_pitch_v[6] = {2.5f,			0.0f,			0.0f,		500.0f,			0.0f,		0.0f};
//float PID_LQR_yaw[6] 		 = {0.1f,			0.0f,			0.0f,		50.0f,			0.0f,		0.0f};
//float PID_LQR_yaw_v[6] 	 = {3.0f,		  0.2f,			0.0f,		130.0f,			5.0f,		0.0f};
//float PID_offset[6] 		 = {1.0f,			0.01f,	  0.0f,		1000.0f,			1000.0f,		0.0f};

//float PID_motor_left[6] 		 = {4.0f,			0.21f,	  0.0f,		13000.0f,			1000.0f,		0.0f};
//float PID_motor_right[6] 		 = {9.0f,			0.9f,	  0.0f,		13000.0f,			1000.0f,		0.0f};


//static void LQR_PIDinit(void)
//{
//	PID_Init(&chassis_center.PID_LQR_x,PID_LQR_x);
//	PID_Init(&chassis_center.PID_LQR_x_v,PID_LQR_x_v);
//	PID_Init(&chassis_center.PID_LQR_pitch,PID_LQR_pitch);
//	PID_Init(&chassis_center.PID_LQR_pitch_v,PID_LQR_pitch_v);
//	PID_Init(&chassis_center.PID_LQR_yaw,PID_LQR_yaw);
//	PID_Init(&chassis_center.PID_LQR_yaw_v,PID_LQR_yaw_v);
//	PID_Init(&chassis_center.PID_offset,PID_offset);
//}

//static void Motor_init(void)
//{
//	PID_Init(&chassis_center.motor_left.speed_pid,PID_motor_left);
//	PID_Init(&chassis_center.motor_right.speed_pid,PID_motor_right);
//}

//void PID_init(void)
//{
//	LQR_PIDinit();
//	Motor_init();
//}
///**
//  * @breif         跟随模式，通过角度环将目标角度转换为目标速度
//  * @param[in]     none
//	* @param[out]    底盘自转速度
//  * @retval        none     
//  */
//static float chassis_follow(void)
//{
//	//云台枪口对应的角度值
//	PID_Calc(&chassis_center.PID_LQR_yaw,chassis_center.CAN_actual_angle,GIMBAL_HEAD_ANGLE);
//	return (float)chassis_center.PID_LQR_yaw.out;
//}


//uint32_t tick,time;

//void chassis_static(void)
//{
//	time=uwTick-tick;
//	tick=uwTick;
//	
//	PID_clear(&chassis_center.PID_LQR_x_v);
//	chassis_center.actual_speed_x=(chassis_center.motor_left.actual_speed-chassis_center.motor_right.actual_speed)*0.000413f/2.0f;
//	chassis_center.actual_pose_x+=chassis_center.actual_speed_x*(float)time*0.001f;
//	if((fabs(chassis_center.actual_speed_x)>0.15f)&&(fabs(chassis_center.actual_speed_x)<0.2f)&&(chassis_center.stop_flag_1==0))
//	{
//		chassis_center.target_pose_x=chassis_center.actual_pose_x;
//		chassis_center.stop_flag_1=1;
//	}
//	if((chassis_center.stop_flag_1)==1)
//	{
//		if((fabs(chassis_center.actual_speed_x)>0.15f)&&(fabs(chassis_center.actual_speed_x)<0.2f)&&(chassis_center.stop_flag_2==0)&&(fabs(chassis_center.pitch_data.IMU_angle)<10.0f))
//		{
//			chassis_center.target_pose_x=chassis_center.actual_pose_x;
//			chassis_center.stop_flag_2=1;
//		}
//	}
//	//	chassis_center.chassis_control_order.gimbal_6020_angle
//	PID_Calc(&chassis_center.PID_LQR_x,chassis_center.target_pose_x,chassis_center.actual_pose_x);
//	PID_Calc(&chassis_center.PID_LQR_x_v,0,chassis_center.actual_speed_x);
//	PID_Calc(&chassis_center.PID_LQR_pitch,0,chassis_center.pitch_data.IMU_angle);
//	PID_Calc(&chassis_center.PID_LQR_pitch_v,0,chassis_center.pitch_data.IMU_speed);
//	chassis_center.follow_err=GIMBAL_HEAD_ANGLE-chassis_center.CAN_actual_angle;
//	if(chassis_center.follow_err>180.0f) chassis_center.follow_err-=360.0f;
//	else if(chassis_center.follow_err<-180.0f) chassis_center.follow_err+=360.0f;
//	PID_Calc(&chassis_center.PID_LQR_yaw,0,-chassis_center.follow_err);

//	if(chassis_center.chassis_control_order.wz_set!=0)
//	{
//		chassis_center.PID_LQR_yaw_v.Kp=1.0f;
//		PID_clear(&chassis_center.PID_LQR_x);
//		chassis_center.target_pose_x=chassis_center.actual_pose_x;
//		PID_clear(&chassis_center.PID_LQR_yaw);
//		PID_Calc(&chassis_center.PID_LQR_yaw_v,chassis_center.chassis_control_order.wz_set,chassis_center.yaw_data.IMU_speed);
//		chassis_center.torque_omega=chassis_center.PID_LQR_yaw_v.out;
//	}
//	else
//	{
//		if(chassis_center.return_flag==1)
//		{
//			chassis_center.PID_LQR_yaw.Kp=1;
//			if(fabs(chassis_center.PID_LQR_yaw.error[0])<5.0f)
//				chassis_center.return_flag=0;
//		}
//		else
//			chassis_center.PID_LQR_yaw.Kp=0.1f;
//			
//		chassis_center.PID_LQR_yaw_v.Kp=3.0f;	
//		PID_Calc(&chassis_center.PID_LQR_yaw_v,chassis_center.PID_LQR_yaw.out,chassis_center.CAN_actual_speed);
//		chassis_center.torque_omega=-chassis_center.PID_LQR_yaw_v.out;
//	}
//	
//	chassis_center.torque_speed=chassis_center.PID_LQR_x.out+chassis_center.PID_LQR_x_v.out;
//	chassis_center.torque_balance=chassis_center.PID_LQR_pitch.out+chassis_center.PID_LQR_pitch_v.out;
//	
//	
//	PID_Calc(&chassis_center.PID_offset,chassis_center.torque_speed+chassis_center.torque_balance,0);
//	chassis_center.motor_left.torque=(-(chassis_center.PID_offset.out+chassis_center.torque_omega));
//	chassis_center.motor_right.torque=((chassis_center.PID_offset.out-chassis_center.torque_omega));
//	
//	chassis_center.motor_left.target_speed += chassis_center.motor_left.torque*0.79334f;
//	chassis_center.motor_right.target_speed += chassis_center.motor_right.torque*0.79334f;
//	
//}

//float last_angle;
//void chassis_normal(void)
//{
//	time=uwTick-tick;
//	tick=uwTick;
//	
//	PID_clear(&chassis_center.PID_LQR_x);
//	chassis_center.stop_flag_1=0;
//	chassis_center.stop_flag_2=0;
//	chassis_center.actual_pose_x=0;
//	chassis_center.target_pose_x=0;
//	chassis_center.actual_speed_x=(chassis_center.motor_left.actual_speed-chassis_center.motor_right.actual_speed)*0.000413f/2.0f;
//	if(fabs(chassis_center.PID_LQR_yaw.error[0])>40.0f)
//		chassis_center.target_speed_x=0;
////	else
////		chassis_center.target_speed_x=arm_cos_f32((chassis_center.PID_LQR_yaw.error[0])/180.0f*PI)*chassis_center.target_speed_x;
//	PID_Calc(&chassis_center.PID_LQR_x_v,chassis_center.target_speed_x,chassis_center.actual_speed_x);
//	PID_Calc(&chassis_center.PID_LQR_pitch,0,chassis_center.pitch_data.IMU_angle);
//	PID_Calc(&chassis_center.PID_LQR_pitch_v,0,chassis_center.pitch_data.IMU_speed);
//	chassis_center.follow_err=GIMBAL_HEAD_ANGLE-chassis_center.CAN_actual_angle;
//	if(chassis_center.follow_err>180.0f) chassis_center.follow_err-=360.0f;
//	else if(chassis_center.follow_err<-180.0f) chassis_center.follow_err+=360.0f;
//	PID_Calc(&chassis_center.PID_LQR_yaw,0,-chassis_center.follow_err);
//	
//	
//	if(chassis_center.chassis_control_order.wz_set!=0)
//	{
//		PID_clear(&chassis_center.PID_LQR_x);
//		chassis_center.target_pose_x=chassis_center.actual_pose_x;
//		PID_clear(&chassis_center.PID_LQR_yaw);
//		chassis_center.PID_LQR_yaw.out=chassis_center.chassis_control_order.wz_set;
//		PID_Calc(&chassis_center.PID_LQR_yaw_v,chassis_center.PID_LQR_yaw.out,chassis_center.yaw_data.IMU_speed);
//		chassis_center.torque_omega=chassis_center.PID_LQR_yaw_v.out;
//	}
//	else
//	{
//		PID_Calc(&chassis_center.PID_LQR_yaw_v,chassis_center.PID_LQR_yaw.out,chassis_center.CAN_actual_speed);
//		chassis_center.torque_omega=-chassis_center.PID_LQR_yaw_v.out;
//	}
//	
//	chassis_center.torque_speed=chassis_center.PID_LQR_x_v.out;
//	chassis_center.torque_balance=chassis_center.PID_LQR_pitch.out+chassis_center.PID_LQR_pitch_v.out;
//	chassis_center.torque_omega=-chassis_center.PID_LQR_yaw_v.out;
//	
//	PID_Calc(&chassis_center.PID_offset,chassis_center.torque_speed+chassis_center.torque_balance,0);
//	chassis_center.motor_left.torque=(-(chassis_center.PID_offset.out+chassis_center.torque_omega));
//	chassis_center.motor_right.torque=((chassis_center.PID_offset.out-chassis_center.torque_omega));
//	
//	chassis_center.motor_left.target_speed += chassis_center.motor_left.torque*0.79334f;
//	chassis_center.motor_right.target_speed += chassis_center.motor_right.torque*0.79334f;
//	
//}


//void chassis_balance(void)
//{
//	PID_clear(&chassis_center.PID_LQR_x);
//	chassis_center.stop_flag_1=0;
//	chassis_center.target_pose_x=0;
//	chassis_center.actual_pose_x=0;
//	chassis_center.actual_speed_x=(chassis_center.motor_left.actual_speed-chassis_center.motor_right.actual_speed)*0.000413f/2.0f;
//	PID_Calc(&chassis_center.PID_LQR_pitch,0,chassis_center.pitch_data.IMU_angle);
//	PID_Calc(&chassis_center.PID_LQR_pitch_v,0,chassis_center.pitch_data.IMU_speed);
//	PID_Calc(&chassis_center.PID_LQR_x_v,0,chassis_center.actual_speed_x);
//	
//	chassis_center.torque_speed=chassis_center.PID_LQR_x_v.out;
//	chassis_center.torque_balance=chassis_center.PID_LQR_pitch.out+chassis_center.PID_LQR_pitch_v.out;
//	chassis_center.torque_omega=0;
//	
//	PID_Calc(&chassis_center.PID_offset,chassis_center.torque_speed+chassis_center.torque_balance,0);
//	chassis_center.motor_left.torque=(-(chassis_center.PID_offset.out+chassis_center.torque_omega));
//	chassis_center.motor_right.torque=((chassis_center.PID_offset.out-chassis_center.torque_omega));
//	
//	chassis_center.motor_left.target_speed += chassis_center.motor_left.torque*0.79334f;
//	chassis_center.motor_right.target_speed += chassis_center.motor_right.torque*0.79334f;
//	
//}

//void chassis_slipped(void)
//{
//	PID_clear(&chassis_center.PID_LQR_x_v);
//	PID_clear(&chassis_center.PID_LQR_x);
//	chassis_center.stop_flag_1=0;
//	chassis_center.target_pose_x=0;
//	chassis_center.actual_pose_x=0;
//	PID_Calc(&chassis_center.PID_LQR_pitch,0,chassis_center.pitch_data.IMU_angle);
//	PID_Calc(&chassis_center.PID_LQR_pitch_v,0,chassis_center.pitch_data.IMU_speed);
//	PID_clear(&chassis_center.PID_LQR_yaw);
//	PID_Calc(&chassis_center.PID_LQR_yaw_v,chassis_center.PID_LQR_yaw.out,chassis_center.CAN_actual_speed);
//	
//	chassis_center.torque_balance=chassis_center.PID_LQR_pitch.out+chassis_center.PID_LQR_pitch_v.out;
//	chassis_center.torque_omega=-chassis_center.PID_LQR_yaw_v.out;
//	
//	PID_Calc(&chassis_center.PID_offset,chassis_center.torque_balance,0);
//	chassis_center.motor_left.torque=(-(chassis_center.PID_offset.out+chassis_center.torque_omega));
//	chassis_center.motor_right.torque=((chassis_center.PID_offset.out-chassis_center.torque_omega));
//	
//	chassis_center.motor_left.target_speed += chassis_center.motor_left.torque*0.79334f;
//	chassis_center.motor_right.target_speed += chassis_center.motor_right.torque*0.79334f;
//	
//}

//float x_err,x_speed_err,pitch_err,pitch_speed_err,yaw_speed_err;
//float x_out,x_v_out,pitch_out,pitch_v_out,yaw_out,target_positon,actual_position;
//float x_par=30.0f,x_v_par=70.74f,pitch_par=15.f,pitch_v_par=2.7f,yaw_par=1.f,kp=-1.0f;
//void LQR_Controller(void)
//{
//	uint8_t i;
//////	if((fabs(pitch_data.IMU_speed)<115)&&(fabs(pitch_data.IMU_angle-1.f)<))
////	target_positon+=chassis_control_order.vx_set*0.01;
////	actual_position+=(motor_left.actual_speed-motor_right.actual_speed)*0.000413f/2.0f*0.007f;
////	x_err=target_positon-actual_position;
//	x_speed_err=chassis_center.chassis_control_order.vx_set-(chassis_center.motor_left.actual_speed-chassis_center.motor_right.actual_speed)*0.000413f/2.0f;
//		pitch_err=1.f-chassis_center.pitch_data.IMU_angle;

//	pitch_speed_err=-chassis_center.pitch_data.IMU_speed;
//	yaw_speed_err=chassis_center.chassis_control_order.wz_set-chassis_center.yaw_data.IMU_speed;
////	LQR_A_B_calc();  
////	P_calc();
////	K_calc();
////	
////	for(i=0;i<12;i++)
////		K_data[i]=-K_data[i];
////	time=uwTick-tick;
//	x_v_out=x_v_par*x_speed_err;
//	x_out=x_par*x_err;
//	pitch_out=pitch_par*pitch_err;
//	pitch_v_out=pitch_v_par*pitch_speed_err;
//	yaw_out=yaw_par*yaw_speed_err;
//	chassis_center.motor_left.torque=(-(x_v_out+pitch_out+pitch_v_out+yaw_out));
//	chassis_center.motor_right.torque=((x_v_out+pitch_out+pitch_v_out-yaw_out));
//	
////	motor_Left.target_current=(int16_t)(motor_Left.torque*195.3125f);
////	motor_Right.target_current=(int16_t)(motor_Right.torque*195.3125f);
////	if(abs(motor_Left.target_current)>1000)
////		motor_Left.target_current=1000;
////	if(abs(motor_Right.target_current)>1000)
////	motor_Right.target_current=1000;
//	chassis_center.motor_left.target_speed += chassis_center.motor_left.torque*0.79334f;
//	chassis_center.motor_right.target_speed += chassis_center.motor_right.torque*0.79334f;
//	PID_Calc(&chassis_center.motor_left.speed_pid,chassis_center.motor_left.target_speed,chassis_center.motor_left.actual_speed);
//	PID_Calc(&chassis_center.motor_right.speed_pid,chassis_center.motor_right.target_speed,chassis_center.motor_right.actual_speed);

//}
