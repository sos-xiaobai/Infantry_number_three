/**
  ******************************************************************************
  * @file    Project/USER/stm32f4xx_it.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   中断文件
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include <string.h>
#include "stdint.h"
#include "stm32f4xx_it.h"
#include <jansson.h>
#include "json.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "can.h"
#include "motor.h"
#include "speed_pid.h"
#include "kinematic.h"
#include "imuReader.h"
#include "bsp_imu_usart.h"


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                         */
/******************************************************************************/
	/**
  * @brief  串口2中断服务函数
  * @param  None
  * @retval None
  */
/*void DEBUG_USART_IRQHandler(void)
{
	uint8_t ucTemp;
//	uint8_t mbox;
//	uint16_t i;
	if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)//接收中断，接收区非空，表示已接收到数据
  {
		ucTemp = USART_ReceiveData( DEBUG_USART );
		switch(ucTemp)
		{
			case 'f':
			{
				BaseVel_To_WheelVel(0, 50, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 'b':
			{
				BaseVel_To_WheelVel(0, -50, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 'l':
			{
				BaseVel_To_WheelVel(-50, 0, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 'r':
			{
				BaseVel_To_WheelVel(50, 0, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			case 's':
			{
				BaseVel_To_WheelVel(0, 0, 0);
				set_motor_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
				break;
			}
			default:
				break;
		}
	}
	if(USART_GetITStatus(DEBUG_USART, USART_IT_TXE) != RESET)//发送区已空，表示可以继续发送数据
  {
		
  }
	USART_ClearITPendingBit(DEBUG_USART,USART_IT_ORE);
}*/

//json字符串接收标志位，第一位为
__IO uint8_t flag_json_recieve = 0x00;
const uint8_t flag_json_start   = 0x01;
const uint8_t flag_json_start_mask   = 0xFE;
const uint8_t flag_json_stop    = 0x02;
const uint8_t flag_json_stop_mask    = 0xFD;
//接收缓冲区__IO


char receiveBuffer[MAX_LENGTH];
char json_Buffer[MAX_LENGTH];
uint8_t flag_command_recieved = 0;
uint8_t flag_command_recieved1 = 0;
uint8_t flag_command_recieved2 = 0;
uint8_t flag_command_recieved3 = 0;
uint8_t flag_command_recieved4 = 0;
uint8_t flag_command_recieved5 = 0;

static char ucTemp;
union Recive_data{int sint_data[2];char char_data[8];}resive_data;

void JSON_USART_IRQHandler(void)
{
	static int i=0;
	
	static uint8_t start_recive_flag=0;
	if(USART_GetITStatus(JSON_USART, USART_IT_RXNE))//接收中断，接收区非空，表示已接收到数据
  {
		ucTemp = USART_ReceiveData(JSON_USART);
		//Usart_SendByte(JSON_USART,ucTemp);
		//主要数据接收
		if(start_recive_flag == 1)
		{
			resive_data.char_data[i] = ucTemp;
			i++;
		}
		if(ucTemp == '*')	//开始标志位
		{
			start_recive_flag=1;
		}
		else if(ucTemp == ';')	//结束标志位
		{
//			if(resive_data.sint_data[0] == 8888)
//			{
//			
//			}
			//printf("接收成功");
			//云台模式
			//control_mode  = resive_data.sint_data[0];
			//底盘速度
			//Kinematics.wheel1.target_speed.linear_vel =  resive_data.sint_data[1]; 
	    //Kinematics.wheel2.target_speed.linear_vel =  resive_data.sint_data[1];
			//yaw
			Kinematics.yaw.target_angle = ((float)(resive_data.sint_data[0]))/100;
			//pitch
			Kinematics.pitch.target_angle = ((float)(resive_data.sint_data[1]))/100;
			//波弹轮
			//Kinematics.trigger.target_angular =  (float)(resive_data.sint_data[4]);
			//摩擦轮
			//Kinematics.fric.target_angular = (float)(resive_data.sint_data[5]);
			start_recive_flag=0;
			i = 0;
		}
		
	}
	/*
	if(USART_GetITStatus(JSON_USART, USART_IT_TXE) != RESET)//发送区已空，表示可以继续发送数据
  {
		
  }
	USART_ClearITPendingBit(JSON_USART,USART_IT_ORE);
	*/
}




union {char data[2];uint16_t uivalue;int16_t ivalue;}tmp;

#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)	//解析传回数据*****需增加
{
  	CanRxMsg RxMessage;
    CAN_Receive(CAN1, 0, &RxMessage);
	uint16_t angle;
	int16_t speed,current;//temp
	
	tmp.data[0] = RxMessage.Data[1];
	tmp.data[1] = RxMessage.Data[0];
	angle = tmp.uivalue;//(RxMessage.Data[0]<<8 | RxMessage.Data[1]);
	tmp.data[0] = RxMessage.Data[3];
	tmp.data[1] = RxMessage.Data[2];
	speed = tmp.ivalue;//(RxMessage.Data[2]<<8 | RxMessage.Data[3]);
	current = (RxMessage.Data[4]<<8 | RxMessage.Data[5]);
	//temp = RxMessage.Data[6];
	
	switch(RxMessage.StdId)
	{
		case(CAN_3508Motor1_ID):
			record_motor_callback(&motor1, angle, speed, current);
		break;
		case(CAN_3508Motor2_ID):
			record_motor_callback(&motor2, angle, speed, current);
		break;
		case(CAN_3508Motor3_ID):
			record_motor_callback(&motor3, angle, speed, current);
		break;
		case(CAN_3508Motor4_ID):
			record_motor_callback(&motor4, angle, speed, current);
		break;
		case(CAN_TRIGGER_ID):
			record_motor_callback(&motor5, angle, speed, current);
		break;
		case(CAN_GIMBAL_Y_ID):
		  record_motor_callback(&gimbal_y, angle, speed, current);
		break;
		case(CAN_GIMBAL_P_ID):
		  record_motor_callback(&gimbal_p, angle, speed, current);
		break;
		case(CAN_LoopBack_ID):
		{
			loopback.motor1_current= RxMessage.Data[0]<<8 | RxMessage.Data[1]<<0;
			loopback.motor2_current= RxMessage.Data[2]<<8 | RxMessage.Data[3]<<0;
			loopback.motor3_current= RxMessage.Data[4]<<8 | RxMessage.Data[5]<<0;
			loopback.motor4_current= RxMessage.Data[6]<<8 | RxMessage.Data[7]<<0;
			
			if(loopback.motor1_current>16384)
				loopback.motor1_current = loopback.motor1_current - 0x10000;
			if(loopback.motor2_current>16384)
				loopback.motor2_current = loopback.motor2_current - 0x10000;
			if(loopback.motor3_current>16384)
				loopback.motor3_current = loopback.motor3_current - 0x10000;
			if(loopback.motor4_current>16384)
				loopback.motor4_current = loopback.motor4_current - 0x10000;
		}
		break;
		default:break;
	}
	
	
}
#endif

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
	
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}
	
//USART6	VL53L1
//unsigned char left_ucRxData[100];
//unsigned char left_ucRxFinish=0;
//static unsigned char left_ucCnt=0;
//void IMU_USART_IRQHandler(void){
//	unsigned char temp=0;
//	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  
//		{
//		  temp=USART_ReceiveData(USART6);
//			left_ucRxData[left_ucCnt++]=temp;
//			if(temp=='m') 
//			{ 
//				left_ucRxFinish=1;
//			  left_ucCnt=0;
//			}				
//		}
//	  USART_ClearITPendingBit(USART6, USART_IT_RXNE);
//}
void USART6_IRQHandler(void)
	{
	
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	

		unsigned char ucData = USART_ReceiveData(USART6);
		ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
		if (ucRxBuffer[0]!=0x59) //数据头不对，则重新开始寻找0x55数据头
		{
			ucRxCnt=0;
			return;
		}
		if (ucRxCnt<9) {return;}//数据不满11个，则返回
		else
		{   
			if(ucRxBuffer[1]==0x59)
			{  	
			  left_distance=(ucRxBuffer[3]|ucRxBuffer[2]);
			}
		}
			ucRxCnt=0;//清空缓存区
		}
	
void UART8_IRQHandler(void)
	{
	
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	

		unsigned char ucData = USART_ReceiveData(UART8);
		ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
		if (ucRxBuffer[0]!=0x59) //数据头不对，则重新开始寻找0x55数据头
		{
			ucRxCnt=0;
			return;
		}
		if (ucRxCnt<9) {return;}//数据不满11个，则返回
		else
		{   
			if(ucRxBuffer[1]==0x59)
			{  	
			  right_distance=(ucRxBuffer[3]|ucRxBuffer[2]);
			}
			}
			ucRxCnt=0;//清空缓存区
		}
//UART8 VL53L1
//unsigned char right_ucRxData[100];
//unsigned char right_ucRxFinish=0;
//static unsigned char right_ucCnt=0;
//void UART8_IRQHandler(void){
//	unsigned char temp=0;
//	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET)  
//		{
//		  temp=USART_ReceiveData(UART8);
//			right_ucRxData[right_ucCnt++]=temp;
//			if(temp=='m') 
//			{ 
//				right_ucRxFinish=1;
//			  right_ucCnt=0;
//			}				
//		}
//	  USART_ClearITPendingBit(UART8, USART_IT_RXNE);
//}

///**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @retval None
//  */
//void SVC_Handler(void)
//{}

///**
//  * @brief  This function handles PendSV_Handler exception.
//  * @param  None
//  * @retval None
//  */
//void PendSV_Handler(void)
//{}

///**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @retval None
//  */
//void SysTick_Handler(void)
//{}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
