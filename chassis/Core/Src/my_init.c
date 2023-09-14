#include "my_init.h"

float Left_current_Pid[6] 			= {1.0f,			0.0f,			0.0f,		29500.0f,			9800.0f,		0.0f};     //速度环
float Right_current_Pid[6] 			= {1.0f,			0.0f,			0.0f,		29500.0f,			9800.0f,		0.0f};     //速度环
extern uint8_t sbus_rx_buffer[36];
extern uint8_t rec_super;

void all_init(void)
{
	motor_pid_init(&(chassis_center.pid));
	motor_pid_init(&(chassis_motor1.pid));
	motor_pid_init(&(chassis_motor2.pid));
	motor_pid_init(&(chassis_motor3.pid));
	motor_pid_init(&(chassis_motor4.pid));
	referee_usart_fifo_init();
	remote_control_init();
	pid_init();
	POWER_PID_Init(&p_pid);
	BUFFER_PID_Init(&b_pid);
	bsp_can_init();
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	init_referee_struct_data();
	delay_init();
	IMU_Init();
	cali_param_init();
	HAL_Delay(1000);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);
//	HAL_UART_Receive_IT(&huart1,&rec_super,1);
  uart_init();
}

void mode_init(void)
{
	chassis_center.pid.loop_flag=POSITION_LOOP;
	chassis_motor1.pid.loop_flag=SPEED_LOOP;
	chassis_motor2.pid.loop_flag=SPEED_LOOP;
	chassis_motor3.pid.loop_flag=SPEED_LOOP;
	chassis_motor4.pid.loop_flag=SPEED_LOOP;
	chassis_center.switch_mode_angle=0;
}