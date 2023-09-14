//#include "bsp_uart.h"
//#include "gimbal_task.h"
//#include <stdio.h>
//#include <string.h>
//#include "dma.h"
//#include "usart.h"
//#include "vision_task.h"
//#include "shoot_task.h"
//#include "can_receive.h"

//extern __IO uint32_t uwTick;
//int windwill_flag=0;//大风车击打标志位 0 自瞄 1 大风车
///*-----------------------------------*/
//int nav_flag=0;   //导航标志位
//int shoot_flag=0; //发射标志位
//int cruise_flag=0;//巡航标志位
//int top_flag=0; //小陀螺标志位 0 没开 1 开了

//int traget_exit_flag=0;//控制权标志位 0 控制权交给下位机  1 控制权交给上位机


///*-----------------------------------*/

//int game_progress;
//int stage_remain_time;
//int red_outpost_HP; 
//int self_remain_HP;
//int shot_flag;

//int shoot_vel=0;
//int auto_exposure_flag=0;
//int reboot_flag=1;
///*-----------------------------------*/

////union Receive_data{int get_data[2];char char_data[8];}receive_data;

//char receive_data[40];
//union Send_Data{int data[12];char char_data[8];} send_data;
//uint8_t start_receive_flag = 0;

//short int F_move_flag=0;		// 四个方向的标识符
//short int L_move_flag=0;
//float VISION_SHOOT_TIME=0.0f;
//static int i = 0;
//float a,b = 0;

//DIRECTION WIND_DIRECTION=CLOCKWISE;

//uint8_t ch;
//int int_get[2];
//int iii=-1;
//uint8_t dma_rx_buff[40];

//void uart_init(void)
//{
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN); 	
//}
//void SHOOT_DIRECTION(void)
//{
//	vision_sent.yaw.target_angle=vision_yaw;
//	vision_sent.pitch.target_angle=vision_pitch;
//    if(F_move_flag==1) //右上
//	{
//		vision_sent.yaw.target_angle =vision_yaw+R_UP_YAW;
//		vision_sent.pitch.target_angle =vision_pitch+R_UP_PITCH;
//	}
//	if(F_move_flag==2) //左上
//	{
//		vision_sent.yaw.target_angle =vision_yaw+L_UP_YAW;
//		vision_sent.pitch.target_angle =vision_pitch+L_UP_PITCH;
//	}
//	if(F_move_flag==3) //左下
//	{
//		vision_sent.yaw.target_angle =vision_yaw+L_DOWN_YAW;
//		vision_sent.pitch.target_angle =vision_pitch+L_DOWN_PITCH;
//	}
//	if(F_move_flag==4) //右下
//	{
//		vision_sent.yaw.target_angle =vision_yaw+R_DOWN_YAW;
//		vision_sent.pitch.target_angle =vision_pitch+R_DOWN_PITCH;
//	}
////	F_move_flag=0;
//}

//uint8_t length=0;
//uint32_t tick,time;
//int vision_t=0,bsp_vision_flag=0;
//void USART1_IRQHandler(void)
//{
//	int j;
//	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//		HAL_UART_DMAStop(&huart1);
//		length=DMA_REC_LEN-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
//		for(j=0;j<length;j++)
//		{
//			if(dma_rx_buff[j] == 0x3b)//结束接收
//			{
//				//					if(bsp_vision_flag==0)
//				////					{
//				////						F_move_flag=(int16_t)(receive_data[1]<<8|receive_data[0]);
//				//					    F_move_flag++;
//				////					    VISION_SHOOT_TIME=(int16_t)(receive_data[3]<<8|receive_data[2]);
//				////						bsp_vision_flag=1;
//				////						if(VISION_SHOOT_TIME<=40) {F_move_flag=0;VISION_SHOOT_TIME=0;bsp_vision_flag=0;}
//				//					}
//	/*---------------------------2022.1.24---------------------------*/

//				
//        
//				time=uwTick-tick;
//				tick=uwTick;
//				
//				//yaw
//	      vision_sent.yaw.target_angle =	(int)(receive_data[3]<<24|receive_data[2]<<16|receive_data[1]<<8|receive_data[0])/100.0f;
//		    
//				//pitch		
//				vision_sent.pitch.target_angle =(int)(receive_data[7]<<24|receive_data[6]<<16|receive_data[5]<<8|receive_data[4])/100.0f;
//				
//				
//				traget_exit_flag=(int)(receive_data[11]<<24|receive_data[10]<<16|receive_data[9]<<8|receive_data[8]);
//				
//				
//				
//				
////				//front
////				vision_sent.chassis.speed_x = (int)(receive_data[11]<<24|receive_data[10]<<16|receive_data[9]<<8|receive_data[8])/100.0f;
////	
////				//go_left
////				vision_sent.chassis.speed_y = (int)(receive_data[15]<<24|receive_data[14]<<16|receive_data[13]<<8|receive_data[12])/100.0f;
////	
////				//turn_left
////				vision_sent.chassis.turn_left = (int)(receive_data[19]<<24|receive_data[18]<<16|receive_data[17]<<8|receive_data[16])/100.0f;
////	
////				//nav_flag
////				nav_flag = (int)(receive_data[23]<<24|receive_data[22]<<16|receive_data[21]<<8|receive_data[20]);
////	
////				//top_flag
////				top_flag= (int)(receive_data[27]<<24|receive_data[26]<<16|receive_data[25]<<8|receive_data[24]);
////			
////				//shoot_flag
////			  shoot_flag=(int)(receive_data[31]<<24|receive_data[30]<<16|receive_data[29]<<8|receive_data[28]);
////				//vision_mode = (int16_t)(receive_data[35]<<24|receive_data[34]<<16|receive_data[33]<<8|receive_data[32]);
////				//cruise_flag
////				cruise_flag=(int)(receive_data[35]<<24|receive_data[34]<<16|receive_data[33]<<8|receive_data[32]);

///*----------------------------------------------------------------*/		
//				
//				//					SHOOT_DIRECTION();
//				start_receive_flag = 0;
//				i = 0;
//				break;
//			}
//			if(start_receive_flag == 1)   //进行数据转移
//			{
//				if(i<31)
//				{
//					receive_data[i]=dma_rx_buff[j];
//					i++;
//				}
//			}
//			if(dma_rx_buff[j] == 0x2a)//开始接收
//			{
//				start_receive_flag = 1;
//			}

//		}
//		HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
//	}
//}

//char Buffer[40],status;
//VISION_RESET VISION_RESET_FLAG=OFF;
//int16_t Sent_dataA = 0;
//int16_t Sent_dataB = 0;
//int16_t Sent_dataC = 0;
//int16_t Sent_dataD = 0;
//int16_t Sent_dataE = 0;
//int16_t Sent_dataF = 0;
//int16_t Sent_dataG = 0;
//int16_t Sent_dataH = 0;
//int16_t Sent_dataI = 0;
//float CAN_angle=0;
//void DMA_Send(void)
//{ 
//	//c=100,d=200;
////	vision_getSpeed();
//	CAN_angle=(gimbal_y.CAN_actual_angle*360.0f/8192.0f)+imu_can_error_y;
//	if(CAN_angle>180)
//		CAN_angle -= 360.0f;
//	if(CAN_angle<-180)
//		CAN_angle += 360.0f;	 
//	
//	if(fricspeed==FRIC_MIN)shoot_vel=15;
//	else shoot_vel=30;

////	Sent_dataA=(uint16_t)(CAN_angle*100.0f);
//	Sent_dataA=gimbal_y.IMU_actual_angle*100.0f;
//	Sent_dataB=gimbal_p.IMU_actual_angle*100.0f;
//	Sent_dataC=shoot_vel*100;
//	Sent_dataD=auto_exposure_flag;
//	Sent_dataE=reboot_flag;
////	Sent_dataA=10*100.0f;
////	Sent_dataB=-10*-100.0f;

////	Sent_dataC=gimbal_y.IMU_actual_speed*100.0f;
////	Sent_dataD=gimbal_p.IMU_actual_speed*-100.0f;
////	Sent_dataE=VISION_RESET_FLAG;

////	//当前比赛阶段
////	Sent_dataE = game_progress;
////	
////	//当前阶段剩余时间
////	Sent_dataF = stage_remain_time;
////	
////	//己方前哨站血量
////	Sent_dataG = red_outpost_HP;//blue_outpost_HP,需要先判断自己是红方还是蓝方，我还没写
////	
////	//自身血量
////	Sent_dataH = self_remain_HP;
////	
////	//是否被击打
////	 Sent_dataI= shot_flag;

//	
//	Buffer[0] =0x2a;
//	
//	Buffer[4] = (Sent_dataA>>24);
//	Buffer[3] =  Sent_dataA>>16;
//	Buffer[2] = (Sent_dataA>>8);
//	Buffer[1] =  Sent_dataA&0xff;
//	
//	Buffer[8] = (Sent_dataB>>24);
//	Buffer[7] =  Sent_dataB>>16;
//	Buffer[6] = (Sent_dataB>>8);
//	Buffer[5] =  Sent_dataB&0xff;
//	
//    Buffer[12] = (Sent_dataC>>24);
//	Buffer[11] =  Sent_dataC>>16;	
//    Buffer[10] = (Sent_dataC>>8);
//	Buffer[9] =  Sent_dataC&0xff;
//	
//	Buffer[16] = (Sent_dataD>>24);
//	Buffer[15] =  Sent_dataD>>16;
//	Buffer[14] = (Sent_dataD>>8);
//	Buffer[13] =  Sent_dataD&0xff;
//	
//	Buffer[20] = (Sent_dataE>>24);
//	Buffer[19] =  Sent_dataE>>16;
//	Buffer[18] = (Sent_dataE>>8);
//	Buffer[17] =  Sent_dataE&0xff;
//	
////	Buffer[24] = (Sent_dataF>>24);
////	Buffer[23] =  Sent_dataF>>16;
////	Buffer[22] = (Sent_dataF>>8);
////	Buffer[21] =  Sent_dataF&0xff;
////	
////	Buffer[28] = (Sent_dataG>>24);
////	Buffer[27] =  Sent_dataG>>16;
////	Buffer[26] = (Sent_dataG>>8);
////	Buffer[25] =  Sent_dataG&0xff;
////	
////  Buffer[32] = (Sent_dataH>>24);
////	Buffer[31] =  Sent_dataH>>16;
////	Buffer[30] = (Sent_dataH>>8);
////	Buffer[29] =  Sent_dataH&0xff;

////  Buffer[36] = (Sent_dataI>>24);
////	Buffer[35] =  Sent_dataI>>16;
////	Buffer[34] = (Sent_dataI>>8);
////	Buffer[33] =  Sent_dataI&0xff;

//	
//	
///*-----------------------------------*/
//	

////  for(int i=1;i<(7-1);i++)
////	{
////	  Buffer[i] = send_data.data[i-1];
////	}
//	Buffer[21] = 0x3b;
//	status=HAL_UART_Transmit(&huart1,Buffer,22,0xff);

//}
