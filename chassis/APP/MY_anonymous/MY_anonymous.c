//#include "MY_anonymous.h"
//#include "chassis_move.h"
//#include "lqr.h"
//#include "imu.h"
//int data_to_send[18];

///*------------------------------------------------------------------------------------------------------
//【函    数】老梁的蓝牙传输函数(两个函数组合）
//【功    能】                
//【参    数】
//【返 回 值】
//【注意事项】                         
//-------------------------------------------------------------------------------------------------------*/
//float power_;
//uint16_t buffer_,power_max_;
//void Bluetooth_transmission(void)
//{
//	get_chassis_power_and_buffer_and_max(&power_, &buffer_,&power_max_);
//  data_to_send[0]=(int)chassis_center.pitch_data.IMU_angle;//3
//  data_to_send[1]=(int)chassis_center.pitch_data.IMU_speed;//22
//	data_to_send[2]=(int)(chassis_center.target_pose_x*100);//200
//  data_to_send[3]=(int)(chassis_center.actual_pose_x*100);//170
//	data_to_send[4]=(int)(chassis_center.actual_speed_x*100);
//  data_to_send[5]=(int)chassis_center.PID_LQR_x_v.Iout;
//	data_to_send[6]=(int)chassis_center.stop_flag_1;
//	data_to_send[7]=(int)chassis_center.stop_flag_2;
//	data_to_send[8]=(int)chassis_center.chassis_mode;
//	data_to_send[9]=(int)power_;
//  Data_Send_F1(data_to_send,10);
//}


//void Data_Send_F1(int *pst, unsigned char len)
//{
//    unsigned char sum = 0,add=0;
//    unsigned char i;
//    unsigned char data_to_send[45];
//    data_to_send[0] = 0xAA;
//    data_to_send[1] = 0xFF;
//    data_to_send[2] = 0xF1;
//    data_to_send[3] = 2*len;
//    for(i=0;i<len;i++)
//    {
//        data_to_send[2*i+4]=(unsigned char)pst[i];
//        data_to_send[2*i+5]=(unsigned char)(pst[i]>>8);
//    }
//    for(i=0;i<2*len+4;i++)
//    {
//        sum += data_to_send[i];
//			add+=sum;
//    }
//    data_to_send[2*len+4] = sum;
//		data_to_send[2*len+5] = add;
//    HAL_UART_Transmit(&huart1, data_to_send, 2*len+6, 100);
//}

