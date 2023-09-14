#ifndef __gimbal_task_behaviour_H
#define __gimbal_task_behaviour_H

#define PITCH_DEADLINE 0.1
#define YAW_DEADLINE 0.1

#define PITCH_MAX_LIMIT  16
#define PITCH_MIN_LIMIT  -16


void gimbal_control_behaviour(void);


#endif
