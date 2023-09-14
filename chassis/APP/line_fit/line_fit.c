//#include "line_fit.h"
//#include <math.h>
//SLIP_NP_t slip_np;

//float input_bias[NERVE_NUM]=
//{
//	2.214,-2.549,-2.435,0.667
//};
//float input_w1[4][NERVE_NUM]=
//{
//	0.697,-1.502,-0.058,0.429,
//	-0.030,-0.686,0.300,-0.376,
//	-0.926,0.034,-2.181,-0.354,
//	-1.006,-0.440,0.125,-0.018
//};
//float output_bias[2]=
//{
//	-2.546,1.942
//};
//float output_w1[2][NERVE_NUM]=
//{
//	2.113,-1.883,-1.997,0.090,
//	-1.455,1.877,1.888,-0.224
//};


//void SoftmaxFunc(float *src, float *dst, uint8_t length)
//{
//	uint8_t i = 0;
//	float sum = 0.0f,temp[9];
//	for(i=0; i<length; i++){
//			temp[i] = exp(src[i]);
//			sum += temp[i];
//	}
//	for(i=0; i<length; i++){
//			dst[i] = temp[i] / sum;
//	}
//}

//float p_slip,exp_1,output[2],SLIP_PRO[2];
//uint8_t logistic_slip_detect(float IMU_angle,float IMU_speed,float speed,float torque)
//{
//	for(uint8_t i=0;i<NERVE_NUM;i++)
//	{
//		slip_np.Hidden_Layer[i]=IMU_angle*input_w1[0][i]+IMU_speed*input_w1[1][i]+speed*input_w1[2][i]+torque*input_w1[3][i]+input_bias[i];
//		slip_np.Hidden_Layer_output[i]=tanh(slip_np.Hidden_Layer[i]);
//	}
//	
//	output[0]=0;
//	output[1]=0;
//	for(uint8_t i=0;i<NERVE_NUM;i++)
//	{
//		output[0]+=slip_np.Hidden_Layer_output[i]*output_w1[0][i];
//		output[1]+=slip_np.Hidden_Layer_output[i]*output_w1[1][i];
//	}
//	output[0]+=output_bias[0];
//	output[1]+=output_bias[1];
//	SoftmaxFunc(output, SLIP_PRO, 2);
//	if((p_slip>0.5f)&&(slip_flag==0))
//		slip_flag=1;
//	else if(slip_flag==1)
//	{
//		if((fabs(chassis_center.pitch_data.IMU_angle)<3.0f)&&(fabs(chassis_center.pitch_data.IMU_speed)<10.0f))
//			slip_flag=0;
//	}
//	if(slip_flag==1)
//		return 1;
//	else
//		return 2;
//}


