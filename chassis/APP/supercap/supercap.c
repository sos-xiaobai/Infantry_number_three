#include "supercap.h"
#include "referee.h"
#include "usart.h"
//uint16_t power_limit=0;
//uint16_t supercap_volt=0;  //�������ݵ�ѹ
//uint16_t supercap_per=0;   //�������ݵ����ٷֱ�
//uint8_t recvStr[7]={0};  
//uint8_t rec_super=0;
//uint8_t cnt=0;
//uint8_t send_data[5]={0};
//uint8_t chassis_power_flag=1;
///**
//  * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
//  * @param[in]     none 
//	* @param[out]    ����ͬ���ʵ��ַ�
//  * @retval        none     
//  */
//void supercap(void)
//{
//	get_chassis_power_limit(&power_limit);  //��ȡ����ϵͳ����
//	if(power_limit==0) power_limit=45;  //�͹��ʱ���
//	
//	send_data[0]=((uint8_t)(power_limit/100))%10;  //��λ
//  send_data[1]=((uint8_t)(power_limit/10))%10;   //ʮλ
//	send_data[2]=((uint8_t)power_limit)%10;        //��λ
//	send_data[3]= chassis_power_flag;              //�̵�����־λ
//	send_data[4]='#';                              //����֡
//	HAL_UART_Transmit(&huart1, send_data, 5, 100);
//	//����
////  if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_ORE))  //���չ���
////	{
////		__HAL_UART_CLEAR_OREFLAG(&huart1); //�����־λ
////    MX_USART1_UART_Init();		//���³�ʼ��
////	}
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==USART1)  //�ж�Ϊ�������ݴ����ж�
//	{
//				/*���ݴ���*/
//				if(rec_super!='#')
//		{
//			if(cnt<=1)
//		  {
//			  recvStr[cnt]=rec_super;  //��ѹ ����
//			  cnt++;
//		  }			
//		}
//		else
//		{
//			get_supercap_data();
//			cnt=0;
//		}
//    HAL_UART_Receive_IT(&huart1,&rec_super,1);   //�����жϺ��ٴ��ֶ����������ж�
//	}
//}


//void get_supercap_data(void)  //ת��
//{
//	supercap_volt=(uint16_t)recvStr[0];
//	supercap_per=(uint16_t)recvStr[1];
//}

/***********************************************************
*@Brief	���ⲿ���õı���
***********************************************************/
int Supercap_Connection_Status = Supercap_Disconnected;
int Supercap_UART_RX_KeepAlive_Flag = 0;
int Capacitor_State = 0; 					//����ʣ������״̬��0Ϊ�ͣ�1Ϊ��
float supercap_volt = 0; 					//�������ݵ�ѹ
int Power_Mode = 0, Last_Power_Mode = 0;	//����ָʾ���̼���ģʽ
float supercap_per = 0; 					//�������ݵ����ٷֱ�

/***********************************************************
*@Brief	����supercap.c�ļ���ʹ�õı���
***********************************************************/
int Global_Time = 0;

/***********************************************************
*@Brief	����supercap.c��������˳��
***********************************************************/
void Supercap_Trans_RefereeData(void);
void Supercap_Keep_Alive(void);
void PowerMode_Judgement(void);

/***********************************************************
  * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
  * @param[in]     none
	* @param[out]    ����ͬ���ʵ��ַ�
  * @retval        none
***********************************************************/

extern void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power);
extern void canTX_To_Beta_Power_Limit(int Power_Mode);
void supercap(int S_Cnt, int MS_Cnt)
{
	Global_Time = S_Cnt*1000 + MS_Cnt;	//����ȫ��ʱ��
	if (Global_Time%200 == 0)
		Supercap_Keep_Alive();
	if (Global_Time%100 == 0)
	{
		Supercap_Trans_RefereeData();
		if(supercap_reboot_flag==1)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //�Ͽ�����̵���
			else 
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	//	PowerMode_Judgement();
	}
}
/***********************************************************
*@fuction	:Supercap_Keep_Alive
*@brief		:ȷ�ϳ�������״̬
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
extern void UART_TX_Supercap_Connection_Check(int Global_Time);
int Keep_Alive_Time_Cnt;
void Supercap_Keep_Alive(void)
{
	Keep_Alive_Time_Cnt++;	//��ʱ
	UART_TX_Supercap_Connection_Check(Global_Time);	//���Ͳ�ѯ����
	//��bsp_uart.c�Ĵ����ж��У����յ���ѯ������Keep_Alive_Time_Cnt������
	if (Keep_Alive_Time_Cnt > 10)//1.2sû���յ����練����
		Supercap_Connection_Status = Supercap_Disconnected;
	else Supercap_Connection_Status = Supercap_Connected;
}

/***********************************************************
*@fuction	:Supercap_Trans_RefereeData
*@brief		:�򳬵緢�Ͳ���ϵͳ����
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
	uint16_t UART_TX_Power_Max = 0, UART_TX_Buffer = 0;
	fp32 UART_TX_Power = 0;
void Supercap_Trans_RefereeData(void)
{
	//�򳬵緢�͹�����Ϣ

    get_chassis_power_and_buffer_and_max(&UART_TX_Power, &UART_TX_Buffer, &UART_TX_Power_Max);  //��ȡ����ϵͳ����
 // UART_TX_Power_Max = 50;
    UartTX_Super_Capacitor(UART_TX_Power_Max, UART_TX_Power);	//�򳬵緢�͹���������������̵�ǰ����
}

/***********************************************************
*@fuction	:PowerMode_Judgement
*@brief		:���ݳ���״̬���Ƶ��̹���ģʽ
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
void PowerMode_Judgement(void)
{
	//���̹���ģʽ�жϣ��ֳ����Ƿ������������
	if (Supercap_Connection_Status == Supercap_Connected)
	{
		if (supercap_volt > 200)
			Power_Mode = High_Voltage_Mode;
		if (supercap_volt <= 150)
			Power_Mode = Low_Voltage_Mode;
		if (Last_Power_Mode == Low_Voltage_Mode && supercap_volt < 180) Power_Mode = Low_Voltage_Mode;
		
	}
	else 
	{
		Power_Mode = Medium_Voltage_Mode;
		
	}
	//��¼�ϴι���ģʽ
    Last_Power_Mode = Power_Mode;
}