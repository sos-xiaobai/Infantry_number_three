#ifndef __vision_task_H
#define __vision_task_H

typedef enum{
    VISION_ON= 0, //自瞄开
    VISION_OFF,    //
}VISION_t;

typedef struct{
	
	float target_angle;
	float target_speed;
}GIMBAL_VI_t;

//上位机发送的值
typedef struct{
	GIMBAL_VI_t yaw;
	GIMBAL_VI_t pitch;
}VISION_GET_t;

typedef enum
{
	PATROL_ON=0,
	PATROL_OFF,
}PRTROL_STATE;

extern VISION_t vision_mode;
extern VISION_GET_t vision_sent;

extern PRTROL_STATE gimbal_prtrol;
extern PRTROL_STATE chassis_prtrol;

void Vision_Task(void);
extern double vision_actual_speed,vision_actual_angle;
extern int Prtrol_cnt,Prtrol_cnt_flag;
void Protrol_Judge(void);
void vision_getSpeed(void);
#endif
