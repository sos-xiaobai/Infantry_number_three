#ifndef __gimbal_task_behaviour_H
#define __gimbal_task_behaviour_H

#define PITCH_DEADLINE 0.01    //�����������ң��������ģʽ������   ��һ����  pid����Ϊ�ڶ�����
#define YAW_DEADLINE 0.01

#define PITCH_MAX_LIMIT  16
#define PITCH_MIN_LIMIT  -16


void gimbal_control_behaviour(void);
int deadline_judge(float a,int flag);
void fly_pitch_judge(void);
void chassis_imu_data_get(void);

#endif
