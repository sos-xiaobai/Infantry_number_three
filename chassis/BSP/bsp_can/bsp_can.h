#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "chassis_move.h"
#include "referee.h"

#define SPIN_TO_FOLLOW        1
#define MODE_NO_SWITCH     0


#define chassis_data_analyze() \
do{ \
		chassis_control_order.vx_set=gdata[0]; \
    chassis_control_order.vy_set=gdata[1]; \
		chassis_control_order.wz_set=gdata[2];	\
    chassis_center.target_current=gdata[3];\
	}while(0)         

	
#define CAN_3508Motor1_ID    0x201
#define CAN_3508Motor2_ID    0x202
#define CAN_3508Motor3_ID    0x203
#define CAN_3508Motor4_ID    0x204
#define CAN_GIMBAL_Y_ID      0x208
#define GIMBAL_CONTROL_ID_1  0x007
#define GIMBAL_CONTROL_ID_2  0x006
#define LOOP_BACK_ID         0x003
#define CHASSIS_TO_GIMBAL_ID_2         0x009
uint8_t bsp_can_init(void);
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata);
void send_gimbal_data(void);	
void canTX_gimbal_2(int16_t vx,int16_t vy);
void send_gimbal_data_2(void);
void canTX_chassis_imu(void);
uint8_t canTX_gimbal_y(int16_t yaw);
extern uint8_t run_away_flag;
extern uint8_t target_exit;
extern uint8_t supercap_reboot_flag;
extern uint8_t reboot_flag;
#endif

