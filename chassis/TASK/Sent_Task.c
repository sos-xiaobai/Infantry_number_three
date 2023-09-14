#include "sent_task.h"
#include "can.h"
/*
0 zc
1 sd
2 xtl
3 wl
*/
uint8_t canTX_chassis_first(int16_t x,int16_t y,int16_t z,int16_t  current_t)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x007;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=x>>8;
	data[1]=x&0xff;
	data[2]=y>>8;
	data[3]=y&0xff;
	data[4]=z>>8;
	data[5]=z&0xff;
	data[6]=current_t>>8;
	data[7]=current_t&0xff;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
	
	return temp;	
}

uint8_t canTX_chassis_second(int16_t mode)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x006;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=2;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=mode>>8;
	data[1]=mode&0xff;
	HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
	return temp;	
}

uint8_t canTX_gimbal(int16_t pitch,int16_t yaw)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=yaw>>8;
	data[1]=yaw&0xff;
	data[2]=pitch>>8;
	data[3]=pitch&0xff;
	data[6]=0;
	data[7]=0;
	data[4]=0;
	data[5]=0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

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

uint8_t canTX_trigger(int16_t trigger)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x201;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=0;
	data[5]=0;
	data[6]=0;
	data[7]=0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}
