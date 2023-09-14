#include "sent_task.h"
#include "can.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "upper_computer.h"
#include "rc_task.h"
#include "shoot_task.h"
#include "vision_task.h"
#include "can_receive.h"
/*
0 zc
1 sd
2 xtl
3 wl
*/
uint8_t data_[8]={0},state;
//给底盘发送数据(x,v,z速度以及yaw轴电流)
uint8_t canTX_chassis_first(int16_t x,int16_t y,int16_t z,int16_t  current_t)
{
	CAN_TxHeaderTypeDef canFrame;
	
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x007;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data_[0]=x>>8;
	data_[1]=x&0xff;
	data_[2]=y>>8;
	data_[3]=y&0xff;
	data_[4]=z>>8;
	data_[5]=z&0xff;
	data_[6]=current_t>>8;
	data_[7]=current_t&0xff;
	state=HAL_CAN_AddTxMessage(&hcan2, &canFrame, data_, &temp);
	
	return temp;	
}

extern uint8_t chassis_power_flag;
uint8_t canTX_chassis_second(int16_t mode,int16_t vision_mode)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint8_t run_away_flag=0;
	uint32_t temp=0;
	
	if(fight_mode==RUN_AWAY)
		run_away_flag=1;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x006;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=mode>>8;
	data[1]=mode&0xff;
	data[2]=vision_mode>>8;
	data[3]=vision_mode&0xff;
	data[4]=traget_exit_flag;
	data[5]=supercap_reboot_flag;
	data[6]=run_away_flag;
	data[7]=reboot_flag;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
	return temp;	
}

//pitch电机控制
uint16_t maxspeed=400;
uint8_t canTX_gimbal_p(int32_t pitch)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x141;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0xA8;
	data[1]=0;
	data[2]=(uint8_t)(maxspeed);
	data[3]=(uint8_t)(maxspeed>>8);
	data[4]=(uint8_t)((pitch));
	data[5]=(uint8_t)((pitch)>>8);
	data[6]=(uint8_t)((pitch)>>16);
	data[7]=(uint8_t)((pitch)>>24);

	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

uint8_t canTX_gimbal_p_2(int16_t pitch)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x141;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0xA1;
	data[4]=(uint8_t)((pitch));
	data[5]=(uint8_t)((pitch)>>8);


	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}
//摩擦轮以及拨弹轮的数据发送
uint8_t canTX_fric(int16_t left,int16_t right,int16_t trigger)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x200;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=right>>8;
	data[1]=right&0xfff;
	data[2]=left>>8;
	data[3]=left&0xfff;
	data[4]=trigger>>8;
	data[5]=trigger&0xfff;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

extern float shoot_speed;
	int16_t Sent_dataA ;  //yaw
  int16_t Sent_dataB;  //pitch
	int16_t	Sent_dataC ; //bullt_speed 
uint8_t data_[8];
uint8_t canTX_UPPER_COMPUTER(void)  //给上位机发送数据
{
	CAN_TxHeaderTypeDef canFrame;
	
	uint32_t temp=0; 
	
//		CAN_angle=(gimbal_y.CAN_actual_angle*360.0f/8192.0f)+imu_can_error_y;
//	if(CAN_angle>180)
//		CAN_angle -= 360.0f;
//	if(CAN_angle<-180)
//		CAN_angle += 360.0f;	 
//	
//	if(fricspeed==FRIC_MIN)shoot_vel=15;
//	else shoot_vel=30;
	
	Sent_dataA = (int16_t)(gimbal_y.IMU_actual_angle*100.0f);  //yaw
  Sent_dataB = (int16_t)(gimbal_p.IMU_actual_angle*100.0f);  //pitch
	Sent_dataC = (int16_t)(shoot_speed*100.0f); //bullt_speed 
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x011;  //对于上位机我们的ID是0x011
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;

	data_[0]=Sent_dataA&0xff;
	data_[1]=Sent_dataA>>8;
	data_[2]=Sent_dataB&0xff;
	data_[3]=Sent_dataB>>8;
//	data_[4]=Sent_dataC&0xff;
//	data_[5]=Sent_dataC>>8;
//  data_[6]=auto_exposure_flag;
//	data_[7]=reboot_flag;
	
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data_, &temp);
	return temp;	
}
uint8_t speed,reboot_old,mode_old,speed_old;
void canTX_UPPER_COMPUTER_2(void)  //
{
	static uint8_t count=0;
	count++;
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0; 
	
//	if(speed_limit==15)
		speed=0;
//	else if(speed_limit==30)
//		speed=2;

//		CAN_angle=(gimbal_y.CAN_actual_angle*360.0f/8192.0f)+imu_can_error_y;
//	if(CAN_angle>180)
//		CAN_angle -= 360.0f;
//	if(CAN_angle<-180)
//		CAN_angle += 360.0f;	 
//	
//	if(fricspeed==FRIC_MIN)shoot_vel=15;
//	else shoot_vel=30;

	if((vision_mode!=mode_old)||(speed!=speed_old)||(reboot_flag!=reboot_old)||(count==10))
	{
		canFrame.IDE=CAN_ID_STD;
		canFrame.StdId=0x013;  //
		canFrame.RTR=CAN_RTR_DATA;
		canFrame.DLC=8;
		canFrame.TransmitGlobalTime=DISABLE;

		data[0]=vision_on_mode;   //自瞄模式为0，小符1，大符2
		data[1]=reboot_flag;
		data[2]=speed;
		
		HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
		count=0;
	}
	mode_old=vision_mode;
	speed_old=speed;
	reboot_old=reboot_flag;
}
