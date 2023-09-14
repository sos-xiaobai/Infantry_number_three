#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "usart.h"
#include <math.h>
#include "bsp_can.h"
#include "bsp_uart.h"
#include "chassis_move.h"


/***********************************************************
*@Brief	仅供bsp_uart.c文件中使用的变量
***********************************************************/
uint8_t KeepAlive_SentData[4];
//union Receive_data{int get_data[2];char char_data[8];}receive_data;
uint8_t receive_data[40];
uint8_t UART1_Sent_Data[10];
uint8_t UART1_Data_Type;

union Send_Data
{
    int data[12];
    char char_data[8];
} Send_Data;
uint8_t start_receive_flag = 0;

int Count = 0;
float a, b = 0;

uint8_t ch;
int int_get[2];
int iii = -1;
uint8_t dma_rx_buff[20];

void uart_init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
}

uint8_t length;
int temp_1;
int temp_2;
void USART1_IRQHandler(void)
{
    int j;
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) //空闲中断（代表这一帧数据传输完了）
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
			temp_2=__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        length = DMA_REC_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        for(j = 0; j < length; j++)
        {
          if(dma_rx_buff[j] == ';')//结束接收
           {

				switch(dma_rx_buff[1]) //判断是什么类型的数据
				{
					case Super_Cap_RX_Typecode:
						extern float supercap_volt; //超级电容电压
						extern float supercap_per; //超级电容电量百分比
						extern int Capacitor_State;
						Capacitor_State   = dma_rx_buff[2];
						supercap_per  = dma_rx_buff[3];
						supercap_volt = dma_rx_buff[4]/10;
						break;
					case SuperCap_Status_RX_Typecode:
						if (KeepAlive_SentData[0] = dma_rx_buff[2])
							if (KeepAlive_SentData[1] = dma_rx_buff[3])
								if (KeepAlive_SentData[2] = dma_rx_buff[4])
									if (KeepAlive_SentData[3] = dma_rx_buff[5])
									{
										extern int Keep_Alive_Time_Cnt;
										Keep_Alive_Time_Cnt = 0;
									}
						break;
            	}
				//超电数据处理

				//接收完毕，复位
//                memset(receive_data, 0, sizeof(receive_data));
//                start_receive_flag = 0;
//                Count = 0;
//                break;
            }
//            if(start_receive_flag == 1)   //进行数据转移
//            {
//                if(Count < length)
//                {
//                    receive_data[Count] = dma_rx_buff[j];
//                    Count++;

//                }

//            }
			//开始接收
//            if(dma_rx_buff[j] == '*')
//                start_receive_flag = 1;
        }
        memset(dma_rx_buff, 0, sizeof(dma_rx_buff));
        HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
    }
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	HAL_UART_Receive_IT(&huart1, dma_rx_buff, 11);
//}
uint8_t Buffer[11];

void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power)
{
	//小数转换
	int IntIze_Power;
	IntIze_Power = (int) (Power*10);
    Buffer[0] =  '*';	//起始帧
	Buffer[1] =  SuperCap_Power_TX_Typecode;	//功率发送标志位
	//发送功率限制值
    Buffer[2] =  (uint8_t)(Power_Limitation / 100);
	Power_Limitation = Power_Limitation - Buffer[2]*100;
    Buffer[3] =  (uint8_t)(Power_Limitation / 10);
    Buffer[4] =  (uint8_t)(Power_Limitation % 10);
	//发送当前功率
    Buffer[5] =  (uint8_t)(IntIze_Power/1000);
	IntIze_Power=IntIze_Power- Buffer[5]*1000;
	Buffer[6] =  (uint8_t)(IntIze_Power/100);
	IntIze_Power=IntIze_Power- Buffer[6]*100;
	Buffer[7] =  (uint8_t)(IntIze_Power/10);
	Buffer[8] =  (uint8_t)(IntIze_Power%10);
	Buffer[9] = 0; //预留位，暂时没用
	Buffer[10] = ';';	//结束帧
    uint8_t status;
    status = HAL_UART_Transmit(&huart1, Buffer, 11, 0xff);
}

/***********************************************************
*@fuction	:Uart_TX_Supercap
*@brief		:
*@param		:Typecode		发送数据包类型
*@param		:Sent_Data[8]	发送的数据，共8字节
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
void Uart_TX_Supercap(int Typecode, uint8_t Sent_Data[8])
{
	uint8_t Data[11];
	//写入起始帧、结束帧
	Data[0] = '*';
	Data[10] = ';';
	//写入发送数据包类型
	Data[1] = Typecode;
	//写入数据包
	for (int i=2; i<=9; i++)
	Data[i] = Sent_Data[i];
	//数据发送，并用Status监视发送成功与否
	uint8_t status;
    status = HAL_UART_Transmit(&huart1, Data, 11, 0xff);
}


void UART_TX_Supercap_Connection_Check(int Global_Time)
{
	uint8_t Sent_Data[8];
	int Keep_Alive_Typecode = SuperCap_KeepAlive_TX_Typecode;
	Sent_Data[0] = (uint8_t)(Global_Time>>24);
	Sent_Data[1] = (uint8_t)((Global_Time>>16)&0xff);
	Sent_Data[2] = (uint8_t)((Global_Time>>8)&0xff);
	Sent_Data[3] = (uint8_t)(Global_Time&0xff);
	for (int i=0; i<=3; i++)
		KeepAlive_SentData[i] = Sent_Data[i];
	Uart_TX_Supercap(Keep_Alive_Typecode, Sent_Data);
}


void DMA_Send(void)
{
    Buffer[0] = '*';
    Buffer[1] = UART1_Data_Type;
    for (int i = 2; i <= 9; i++)
        Buffer[i] = UART1_Sent_Data[i - 2];
    Buffer[10] = ';';
    uint8_t status;
    status = HAL_UART_Transmit(&huart1, Buffer, 11, 0xff);

    for (int i = 0; i < 8; i++)
        UART1_Sent_Data[i] = 0;
    UART1_Data_Type = 0;
}

