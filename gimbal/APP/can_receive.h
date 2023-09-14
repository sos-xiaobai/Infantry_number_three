#ifndef __can_receive_H
#define __can_receive_H

#include "main.h"

#include "bsp_can.h"
#define YAW_MOTOR_ID	       0x205   //0x205
#define PITCH_MOTOR_ID		   0x141
#define SHOOT_LEFT_MOTOR_ID    0x202
#define SHOOT_RIGH_MOTOR_ID    0x201
#define SHOOT_MOTOR_TRIGGER_ID 0x203
#define CHASSIS_SPEED_ID       0x009
#define REFEREE_DATA_ID        0x006
#define UPPER_COMPUTER_ID      0x010
#define CHASSIS_IMU_ID         0x012

typedef struct
{
	uint16_t    lastangle; /*上次电机角度*/
	
	uint16_t 			sumangle;/*角度和*/
	
	float 		angle;/*电机实际角度*/
	
	int16_t		  speed;/*电机实时速度*/
	
	uint8_t     temp; /*电机温度*/
	
	int32_t 		turns;/*电机从上电开始转过的圈数*/
}Motor_HandleTypeDef;		



extern Motor_HandleTypeDef yaw_can_rx,pitch_can_rx,shoot_can_rx[2];
extern int16_t heat,heat_limit;
extern uint8_t target_exit_rec_flag;
extern uint8_t speed_change_flag,game_status,speed_limit;
#endif
