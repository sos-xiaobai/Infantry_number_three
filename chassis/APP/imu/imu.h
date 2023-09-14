#ifndef __IMU_H
#define __IMU_H

typedef struct{
	float IMU_angle;
	float IMU_speed;
}IMU_data_t;

void IMU_read(void);
#endif
