/**
  ******************************************************************************
  * @file    Project/APP/json.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   json文件
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include <string.h>
#include <stdio.h>
#include "json.h" 
#include "stm32f4xx_it.h"
/********信息发送部分********/
/**
  *@brief 信息发送
  */
extern float left_distance;
extern float right_distance;
extern ext_game_robot_state 			Robot_State;
extern ext_power_heat_data 			Power_Heat_Data;
extern referee_info_type             referee_info;


int gimbal_mode_flag=0;
float end = 0.0;

union Send_Data{int data[2];char char_data[8];} send_data;
void send_infantry_info_by_json(void)//使用这一个
{  	
	send_data.data[0] = (int)(Kinematics.yaw.actual_angle*100);
	send_data.data[1] = (int)(Kinematics.pitch.actual_angle*100);
	
	//printf("发送的数据为：");
	Usart_SendByte(JSON_USART,'*');
	for(int i=0;i<8;i++)
	{
		Usart_SendByte( JSON_USART,send_data.char_data[i]);
	}
	Usart_SendByte(JSON_USART,';');
}

/**
  *@brief 底盘信息发送
  */
void send_chassis_info_by_json(void)
{
	json_t *root;
	char *out;           //
		root = json_pack("[{sfsfsfsfsf}[fffff]]",\
		        "HP", (Robot_State.remain_HP),\
	          "Shoot_Heatlimit", (Robot_State.shooter_heat0_cooling_limit),\
	          "Chassis_Power", (Power_Heat_Data.chassis_power),\
	          "Chassis_PowerBuff", (Power_Heat_Data.chassis_power_buffer),\
	          "Shoot_Heat", (Power_Heat_Data.shooter_heat0),\
	
			      (Robot_State.remain_HP),\
		        (Robot_State.shooter_heat0_cooling_limit),\
	          (Power_Heat_Data.chassis_power),\
	          (Power_Heat_Data.chassis_power_buffer),\
	          (Power_Heat_Data.shooter_heat0));
	/*	"linear_x", (Kinematics.actual_velocities.linear_x),\
					  "yaw_angle", (gimbal_y.actual_angle),\
						"pitch_angle", (gimbal_p.actual_angle),\
	          "left_distance", (left_distance),\
	          "right_distance", (right_distance),\


						(Kinematics.actual_velocities.linear_x),\
						(gimbal_y.actual_angle),\
						(gimbal_p.actual_angle),\
						(left_distance),\
            (right_distance));
*/

	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);
}

/**
  *@brief 云台信息发送
  */
void send_gimbal_info_by_json(void)   
{
  json_t *root;
	char *out;           
	root = json_pack("[{sfsf}[ff]]",\
					  "yaw_angle", (gimbal_y.actual_angle),\
						"pitch_angle", (gimbal_p.actual_angle),\
						(gimbal_y.actual_angle),\
						(gimbal_p.actual_angle));   //(int),
	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);

}

/**
  *@brief 摩擦轮信息发送 
  */
void send_fric_info_by_json()   
{
//����   
Kinematics.fric.actual_angular=7;
json_t *root;
	char *out;           //
	root = json_pack("[{sf}[f]]",\
						
					  "fric_angular", (Kinematics.fric.actual_angular),\
						
						(Kinematics.fric.actual_angular));   //(int),
						
	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);
}

/**********信息解析部分*************/
	float tmp_getx;
	float tmp_gety;

  float pitch_angle;
	float yaw_angle;


/**
  *@brief 总的传输函数，只用了这个
  */
void resolve_json_chassis_command(void)
{
	json_t *root;
	json_t *chassis_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error); //
	chassis_obj = json_object_get( root, "chassis" );  //Get a value corresponding to key from object
	item_obj = json_array_get( chassis_obj, 1 );//Returns the element in array at position index	
	Kinematics.target_velocities.linear_x =0.001f*json_integer_value(item_obj);	//底盘速度
	item_obj = json_array_get( chassis_obj, 6 );//Returns the element in array at position index	
	control_flag = json_integer_value(item_obj);//搜寻控制命令
	if(control_flag==0)//搜寻时屏蔽云台信息
	{
		item_obj = json_array_get( chassis_obj, 0 );//Returns the element in array at position index	
		gimbal_mode_flag =1.0f*json_integer_value(item_obj);	//real
		if(gimbal_mode_flag==0)
		{
			gimbal_loop = speed_loop;
			item_obj = json_array_get( chassis_obj, 2 );
			Kinematics.yaw.target_angular=1.0f*json_integer_value(item_obj); //10000;
			//Kinematics.target_velocities.linear_y = 1.0f*json_integer_value(item_obj);
			item_obj = json_array_get( chassis_obj, 3 );
			Kinematics.pitch.target_angular=1.0f*json_integer_value(item_obj);
			//Kinematics.target_velocities.angular_z = 1.0f*json_integer_value(item_obj);
		}
		if(gimbal_mode_flag==1)
		{
			gimbal_loop = position_loop;
			item_obj = json_array_get( chassis_obj, 2 );
			Kinematics.yaw.target_angle=0.001f*json_integer_value(item_obj); //10000;
			//Kinematics.target_velocities.linear_y = 1.0f*json_integer_value(item_obj);
			item_obj = json_array_get( chassis_obj, 3 );
			Kinematics.pitch.target_angle=0.001f*json_integer_value(item_obj);
			//Kinematics.target_velocities.angular_z = 1.0f*json_integer_value(item_obj);
		}
	}
	item_obj = json_array_get( chassis_obj, 4 );
	Kinematics.trigger.target_angular=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( chassis_obj, 5 );
	Kinematics.fric.target_angular=1.0f*json_integer_value(item_obj);
	json_decref(item_obj); //Decrement the reference count of json. As soon as a call to json_decref() drops the reference count to zero, the value is destroyed and it can no longer be used.
	json_decref(chassis_obj);
	json_decref(root);
}

/**
  *@brief 接受云台数据
  */
void resolve_json_gimbal_speed_command()
{ 
	json_t *root;
	json_t *gimbal_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	gimbal_obj = json_object_get( root, "gimbal" );
	item_obj = json_array_get( gimbal_obj, 0 );
  Kinematics.yaw.target_angle=1.0f*json_integer_value(item_obj); //10000;
	item_obj = json_array_get( gimbal_obj, 1 );
	Kinematics.pitch.target_angle=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(gimbal_obj);
	json_decref(root);
}

/**
*@brief 接受控制指令	，注意上位机程序结束时最后传一个control_flag = 0
	传入的json数据;
{\
    control:\
    [control_flag]\
}
  */
void resolve_json_control_command() //  '>'
{ 
	json_t *root;
	json_t *control_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	control_obj = json_object_get( root, "control" );
	item_obj = json_array_get( control_obj, 0 );
	control_flag = json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(control_obj);
	json_decref(root);
}

/**
  *@brief 接受上位机调节pid参数
  */
/*void resolve_json_pidparam_command(void)   
{ 

}
*/

void resolve_json_gimbal_angle_command(void)//没有用上
{ 
	json_t *root;
	json_t *gimbal_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	gimbal_obj = json_object_get( root, "gimbal_angle" );
	item_obj = json_array_get( gimbal_obj, 0 );
	Kinematics.yaw.target_angle=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( gimbal_obj, 1 );
	Kinematics.pitch.target_angle=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(gimbal_obj);
	json_decref(root);
}

/**
  *@brief 接受摩擦轮数据
  */
int fric_1;
void resolve_json_fric_command()
	{ 
	json_t *root;
	json_t *fric_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	fric_obj = json_object_get( root, "fric_angular" );
	item_obj = json_array_get( fric_obj, 0 );
	Kinematics.fric.target_angular=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(fric_obj);
	json_decref(root);

}
	
/**
  *@brief 接受拨弹轮数据
  */
void resolve_json_trigger_command()
	{ 
	json_t *root;
	json_t *trigger_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	//trigger_obj = json_object_get( root, "trigger_angular" );
	//item_obj = json_array_get( trigger_obj, 0 );
	//Kinematics.trigger.target_angular=1.0f*json_integer_value(item_obj);
		
	trigger_obj	= json_object_get( root, "fire_speed" );
	item_obj = json_array_get( trigger_obj, 0 );
	Kinematics.trigger.target_angular=1.0f*json_integer_value(item_obj);
	item_obj = json_array_get( trigger_obj, 1 );
	Kinematics.fric.target_angular=1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(trigger_obj);
	json_decref(root);

}
	

void resolve_json_mode_command()
{
  resolve_chassis_mode_command();
	resolve_gimbal_mode_command();
	resolve_fric_mode_command();

}
void resolve_chassis_mode_command()
{ 
  json_t *root;
	json_t *chassis_mode_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	chassis_mode_obj = json_object_get( root, "translation" );
	item_obj = json_array_get( chassis_mode_obj, 0 );
	chassis_modes = (chassis_mode_t)(json_integer_value(item_obj));
	json_decref(item_obj);
	json_decref(chassis_mode_obj);
	json_decref(root);

}
void resolve_gimbal_mode_command()
{
  json_t *root;
	json_t *gimbal_mode_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	gimbal_mode_obj = json_object_get( root, "gimbal_mode" );
	item_obj = json_array_get( gimbal_mode_obj, 0 );
	gimbal_modes = (gimbal_mode_t)(json_integer_value(item_obj));
	json_decref(item_obj);
	json_decref(gimbal_mode_obj);
	json_decref(root);

}
void resolve_fric_mode_command()
{
  json_t *root;
	json_t *fric_mode_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(json_Buffer,0,&error);
	fric_mode_obj = json_object_get( root, "fric_mode" );
	item_obj = json_array_get( fric_mode_obj, 0 );
	fric_modes = (fric_mode_t)(json_integer_value(item_obj));
	json_decref(item_obj);
	json_decref(fric_mode_obj);
	json_decref(root);
}




//对上位机传来的信息进行二次加工
/**
  *@brief pwm控制云台模式下的角度和脉宽换算
  */
void caclulate_pwm_pulse()
{
	float unit_pwm_pulse= (840.0f/360.0f);
	
	if(Kinematics.pitch.target_angle< (-90) && Kinematics.pitch.target_angle > 90)
	   pwm_pulse_p = (BASIC_PITCH_ANGLE_PWM +unit_pwm_pulse * Kinematics.pitch.target_angle)*1.0f;
	if(Kinematics.yaw.target_angle < (-90) && (Kinematics.yaw.target_angle > 90))
	   pwm_pulse_y = (BASIC_YAW_ANGLE_PWM +unit_pwm_pulse * Kinematics.yaw.target_angle)*1.0f;
	
}

void caclulate_handpwm_pulse()
{
  static double  yaw_pwm_pulse=1500;
  static double  pitch_pwm_pulse=1500;

	if(Kinematics.yaw.target_angular==1 && pwm_pulse_y>=1395)
	{
    yaw_pwm_pulse=yaw_pwm_pulse-1;
		pwm_pulse_y=yaw_pwm_pulse;
		delay_ms(2);
	}
	
	if(Kinematics.yaw.target_angular==-1 && pwm_pulse_y<=1605)
	{
		yaw_pwm_pulse++;
		pwm_pulse_y=yaw_pwm_pulse;
		delay_ms(2);
		
	}
	if(Kinematics.yaw.target_angular==0)
	{
		if(yaw_pwm_pulse>1500)
			 yaw_pwm_pulse=yaw_pwm_pulse-1;
		   pwm_pulse_y=yaw_pwm_pulse;
		   delay_ms(2);
		if(yaw_pwm_pulse<1500)
		  yaw_pwm_pulse++;
		  pwm_pulse_y=yaw_pwm_pulse;
		  delay_ms(2);
	}
	
	
	if(Kinematics.pitch.target_angular==1 && pwm_pulse_p<=1605)
	{
		pitch_pwm_pulse++;
		pwm_pulse_p=pitch_pwm_pulse;
		delay_ms(2);
		
	}
	if(Kinematics.pitch.target_angular==-1 && pwm_pulse_p>=1395)
	{
		pitch_pwm_pulse=pitch_pwm_pulse-1;
		pwm_pulse_p=pitch_pwm_pulse;
		delay_ms(2);
	}
	
		if(Kinematics.pitch.target_angular==0)
	{
			if(pitch_pwm_pulse>1500)
			 pitch_pwm_pulse=pitch_pwm_pulse-1;
		   pwm_pulse_p=pitch_pwm_pulse;
		   delay_ms(2);
		  if(pitch_pwm_pulse<1500)
		  pitch_pwm_pulse++;
		  pwm_pulse_p=pitch_pwm_pulse;
		  delay_ms(2);
	}

}
