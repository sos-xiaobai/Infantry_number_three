#include "imu.h"
#include "bsp_imu.h"
#include "chassis_move.h"

void IMU_read(void)
{
	chassis_center.yaw_angle=INS_angle[0]/(2*3.141590f)*360.0f;
	chassis_center.yaw_speed=INS_gyro[2]*0.5f/(2*3.141590f)*360.0f;
	
	chassis_center.pitch_angle=-1.00f*INS_angle[2]/(2*3.141590f)*360.0f;
	chassis_center.pitch_speed=1.00f*INS_gyro[1]/(2*3.141590f)*360.0f;
}