#ifndef __gimbal_task_behaviour_H
#define __gimbal_task_behaviour_H

#define PITCH_DEADLINE 0.01    //正常键鼠或者遥控器操作模式的死区   第一死区  pid死区为第二死区
#define YAW_DEADLINE 0.01

#define PITCH_MAX_LIMIT  16
#define PITCH_MIN_LIMIT  -16


void gimbal_control_behaviour(void);
int deadline_judge(float a,int flag);
void fly_pitch_judge(void);
void chassis_imu_data_get(void);

#endif
