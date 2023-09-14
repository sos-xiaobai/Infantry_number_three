#include "can_receive.h"
#include "stm32f4xx.h"

#include "string.h"
#include "bsp_math.h"
#include "led.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "gimbal_task_behaviour.h"

Motor_HandleTypeDef	yaw_can_rx={0},pitch_can_rx={0},shoot_can_rx[2]={0};
int16_t heat,heat_limit;
int16_t euler_angle[3];
uint8_t game_status,speed_limit;
extern int8_t upper_computer_data[8];
extern int8_t chassis_imu_data[8];
float shoot_speed;
uint8_t target_exit_rec_flag;//上位机卡死时使用的判断标志位
float last_shoot_speed;
uint8_t speed_change_flag;
//CAN中断接收函数（整个函数）
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxFrame;
	uint8_t rxData[8]={0};

	led_on(&led[2]);
	if(hcan==&hcan1)
	{
		
		HAL_CAN_GetRxMessage(&hcan1,  CAN_RX_FIFO0, &rxFrame,  rxData);
		switch (rxFrame.StdId)
		{
			case SHOOT_MOTOR_TRIGGER_ID:
							    {
										Trigger_Motor_Callback(&rc_shoot.trigger,(int16_t)rxData[0]<<8|rxData[1],(int16_t) rxData[2]<<8|rxData[3],(int16_t) rxData[4]<<8|rxData[5]);
										break;
								  }//接收拨弹轮数据
			case PITCH_MOTOR_ID:
								{
									if(rxData[0]==0xA8)
									{
										pitch_can_rx.speed = (int16_t) rxData[5]<<8|rxData[4];
										pitch_can_rx.temp = (int16_t) rxData[1];
									}
									break;
								}//接收pitch电机数据	
			case SHOOT_LEFT_MOTOR_ID:
				                    {
									    shoot_can_rx[0].angle = (int16_t) rxData[0]<<8|rxData[1];	
										  shoot_can_rx[0].speed = (int16_t) rxData[2]<<8|rxData[3];
										  rc_shoot.left_fric.actual_speed=shoot_can_rx[0].speed ;
										  break;
									}	
			case SHOOT_RIGH_MOTOR_ID:
									{
										  shoot_can_rx[1].angle = (int16_t) rxData[0]<<8|rxData[1];	
										  shoot_can_rx[1].speed = (int16_t) rxData[2]<<8|rxData[3];
										  rc_shoot.right_fric.actual_speed=shoot_can_rx[1].speed ;
										  break;
									}									
		}
	}

	else if(hcan==&hcan2)
	{

		HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &rxFrame,  rxData);
		switch (rxFrame.StdId)
		{
			case CHASSIS_SPEED_ID:
								 {
										  chassis_speed_x = (int16_t) rxData[0]<<8|rxData[1];	
										  chassis_speed_y = (int16_t) rxData[2]<<8|rxData[3];
										  break;
								 }
       case YAW_MOTOR_ID:
								{
										yaw_can_rx.lastangle = yaw_can_rx.angle;
										yaw_can_rx.speed = (int16_t) rxData[0]<<8|rxData[1];
										yaw_can_rx.angle = (int16_t) rxData[2]<<8|rxData[3];
//										yaw_can_rx.temp = (int16_t) rxData[6];
//													
//										yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)<-4096?yaw_can_rx.turns+1:yaw_can_rx.turns;
//										yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)>4096? yaw_can_rx.turns-1:yaw_can_rx.turns;	
										break;
								}
			 case REFEREE_DATA_ID:
					 {
						 game_status=rxData[0];
						 heat=(int16_t) (rxData[1]<<8|rxData[2]);
						 heat_limit=(int16_t) (rxData[3]<<8|rxData[4]);
						 shoot_speed=((float)(rxData[5]<<8|rxData[6]))/10.0f;
						 if(last_shoot_speed!=shoot_speed)
						 {
							 speed_change_flag=1;
							 last_shoot_speed=shoot_speed;
						 }
						 speed_limit=rxData[7];
						 break;
					 }
			 case UPPER_COMPUTER_ID: //从上位机收到的数据
			     {
						 	target_exit_rec_flag=1;
				     for(int i=0;i<8;i++)
					    {
								upper_computer_data[i]=rxData[i];
					    }
							break;
			     }
			 case CHASSIS_IMU_ID:
			    {
				
							for(int i=0;i<8;i++)
					    {
								chassis_imu_data[i]=rxData[i];
					    }
							break;
			    }					  
		}		
	}
}
