#ifndef __vision_task_H
#define __vision_task_H

#include "main.h"

typedef enum{
    VISION_ON= 0, //自瞄开
    VISION_OFF,    
}VISION_t;

typedef enum{
  AUTO_ON =0,         //三号打符
  WINDWILL_SMALL,
	WINDWILL_BIG
}VISION_MODE_t;

typedef struct{
	
	float target_angle;
	float target_speed;
}GIMBAL_VI_t;


typedef struct{
	float speed_x;//front 
	float speed_y;//go_left
	float turn_left;//turn_left
}CHASSIS_VI_t;


//上位机发送的值
typedef struct{
	GIMBAL_VI_t yaw;
	GIMBAL_VI_t pitch;
	CHASSIS_VI_t chassis;
}VISION_GET_t;

extern VISION_t vision_mode;
extern VISION_GET_t vision_sent;
extern float vision_yaw,vision_pitch; 
extern VISION_MODE_t vision_on_mode;
void Vision_Task(void);
extern double vision_actual_speed,vision_actual_angle;
void vision_getSpeed(void);
#endif
