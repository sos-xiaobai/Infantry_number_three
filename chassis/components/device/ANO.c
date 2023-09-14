/**
  ******************************************************************************
  * @file    ANO.c
  * @author  ZWT
  * @version V1.0
  * @date    2021-xx-xx
  * @brief   匿名数传使用
	*					 使用USART 依据V7通讯协议 编写代码
	*					 串口空闲中断接收 DMA 双缓冲
	*					 串口DMA发送
  ******************************************************************************
  * @attention
  * 平台:		RoboMaster C型开发板 STM32F407IGHx
  * QQ：		2457513659
  * Email:	zwt190315@163.com
  *	@reference 
	*		DJI C型开发板例程
	*		匿名V7通讯协议
  * @function
	*		void DEBUG_strings(const char *pdata,uint8_t color)
	*		void DEBUG_printf(const char *pdata,int32_t PAR_VAL)
  *		void DEBUG_Anony16Send(uint8_t id,int16_t *pst,uint8_t len)
	*		void DEBUG_Anony32Send(uint8_t id,int32_t *pst,uint8_t len)
  * @parament
  *		
  *	
  ******************************************************************************
**/ 
#include "main.h"
#include "ANO.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t ANO_rx_buffer[2][DEBUG_RX_BUF_NUM]={0};
static Debug_receive Debug_data_pack;//参数写入数据结构体

int32_t test[3]={0};
/****************匿名数传串口初始化*******************************************/
/*
 *使用USART1的DMA接收和发送
 */
void ANO_usart_init()
{
	debug_usart_init(ANO_rx_buffer[0],ANO_rx_buffer[1],DEBUG_RX_BUF_NUM);
}
/****************匿名数传发送发送信息*******************************************/
/**
  * @brief         	ANO数传发送字符串信息
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *发送字符串 brg显示颜色 最长不超过255字符
 *b=black=0;	r=rad=1;	g=green=2;
 */

void DEBUG_strings(const char *pdata,uint8_t color)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[256]={0};//数据最长255字节
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = STM32_ADDR;
	data_to_send[count++] = TMS32_LOG_CMD;
	data_to_send[count++] = 0;//数据长度暂时为0
	data_to_send[count++] = color;
	for (i=0;pdata[i]!='\0';i++)
	{
		data_to_send[count++] = pdata[i];
	}
	//data_to_send[count++] = 0;	不需要加字符串结束字符'\0'
	data_to_send[3] = i+1;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	/*DMA发送数据*/
	debug_dma_tx(data_to_send,count);
}
/**
  * @brief         	ANO数传发送字符串+数字信息
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *发送字符串+数字
 */
void DEBUG_printf(const char *pdata,int32_t PAR_VAL)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[256]={0};//数据最长255字节
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = STM32_ADDR;
	data_to_send[count++] = TMS32_LOGS_CMD;
	data_to_send[count++] = 0;//数据长度暂时为0
	memcpy(&data_to_send[4],&PAR_VAL,4);
	count+=4;
	for (i=0;pdata[i]!='\0';i++)
	{
		data_to_send[count++] = pdata[i];
	}
	//data_to_send[count++] = 0; //不需要加字符串结束字符'\0'
	data_to_send[3] = i+4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	/*DMA发送数据*/
	debug_dma_tx(data_to_send,count);
}

/****************匿名数传发送数据*******************************************/
/**
	* @brief         	ANO数传的调试数据发送
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *发送2字节数据 int16型 -32767~+32767
 */
void DEBUG_Anony16Send(uint8_t id,int16_t *pst,uint8_t len)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[46];//数据最长40字节 int16数据20个
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = ANONY_ADDR;
	data_to_send[count++] = id;
	data_to_send[count++] = 0;
	len = len<10 ? len:10; 
	for (i=0;i<len;i++) //0~9 十个数据
	{
		data_to_send[count++] = (unsigned char)pst[i]; 
		data_to_send[count++] = (unsigned char)(pst[i]>>8);  //高8位
	}//小端模式 低字节在前 高字节在后 一次最多10个数据
	data_to_send[3] = count - 4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	/*DMA发送数据*/
	debug_dma_tx(data_to_send,count);
	//HAL_UART_Transmit(&huart1,senddatas,20,1000);
}
/**
  * @brief         	ANO数传的调试数据发送
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
/*
 *发送4字节数据 int32型 非常大 可以除以相应10n次方，转换为小数
 */
void DEBUG_Anony32Send(uint8_t id,int32_t *pst,uint8_t len)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[66];//数据最长60字节 int16数据40个
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = ANONY_ADDR;
	data_to_send[count++] = id;
	data_to_send[count++] = 0;
	len = len<10 ? len:10; 
	for (i=0;i<len;i++)
	{
		data_to_send[count++] = (unsigned char)pst[i]; 			//低8位
		data_to_send[count++] = (unsigned char)(pst[i]>>8);  //高8位
		data_to_send[count++] = (unsigned char)(pst[i]>>16); //高16位
		data_to_send[count++] = (unsigned char)(pst[i]>>24); //高24位
	}//小端模式 低字节在前 高字节在后 一次最多10个数据
	data_to_send[3] = count - 4;
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	debug_dma_tx(data_to_send,count);
}
/**
  * @brief         	ANO数传的参数读取返回
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
static void DEBUG_AnonyWriteReturn(uint8_t *pst)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[9];//参数返回帧格式
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = STM32_ADDR;
	data_to_send[count++] = RETURN_ID;
	data_to_send[count++] = 3;
	data_to_send[count++] = pst[0];
	data_to_send[count++] = pst[1];
	data_to_send[count++] = pst[2];
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	debug_dma_tx(data_to_send,count);
}
/**
  * @brief         	ANO数传的参数读取返回
  * @param[in]      none
  * @retval         none
	*	@attention			
  */
//int32_t LookVAL0=0,LookVAL1=0,LookVAL2=0;
//uint8_t LookArrys[6]={0};
static void DEBUG_AnonyReadReturn(uint16_t PAR_ID,int32_t PAR_VAL)
{
	uint8_t sumcheck = 0,addcheck = 0; 
	uint8_t i,count = 0,data_to_send[12];//参数返回帧格式
	data_to_send[count++] = ANONY_HEAD;
	data_to_send[count++] = PC_ADDR;
	data_to_send[count++] = STM32_WRITE_CMD;
	data_to_send[count++] = 6;
	memcpy(&data_to_send[4],&PAR_ID,2);
	memcpy(&data_to_send[6],&PAR_VAL,4);
	count+=6;
//	data_to_send[count++] = (unsigned char)(PAR_ID&0x00FF);
//	data_to_send[count++] = (unsigned char)(PAR_ID&0xFF00)>>8;
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0x000000FF);
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0x0000FF00)>>8;
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0x00FF0000)>>16;
//	data_to_send[count++] = (unsigned char)(PAR_VAL&0xFF000000)>>24;
//	LookVAL0=PAR_VAL;
//	memcpy(&LookArrys,&data_to_send[4],6);
	for (i=0;i<data_to_send[3]+4;i++)
	{
		sumcheck += data_to_send[i];
		addcheck += sumcheck;
	}
	data_to_send[count++] = sumcheck;
	data_to_send[count++] = addcheck;
	debug_dma_tx(data_to_send,count);
}

/*****************************匿名数传参数接收*****************************/
 /**
  * @brief  Debug串口的数据接收函数
  * @param  接收字节
  * @retval 无
	* @attation
	* @note USART1 空闲中断函数中调用
  */

void DEBUG_Data_solve(uint8_t *pData)
{
	uint8_t sumcheck = 0,addcheck = 0,i; 
	uint8_t return_data[3];
	uint16_t PAR_ID;
	int32_t PAR_VAL;
	if (pData[0]!=ANONY_HEAD) //数据头不对，则重新开始寻找0xAA数据头
		return;
	switch (pData[2])
	{
		/*数据写入指令 返回对应ID的数据*/
		case STM32_WRITE_CMD:
			{
					for (i=0;i<pData[3]+4;i++)
				{
					sumcheck += pData[i];
					addcheck += sumcheck;
				}
				if(sumcheck == pData[10]&&addcheck == pData[11])
				{
					memcpy(&Debug_data_pack,&pData[4],6);	
					return_data[1] = pData[10];
					return_data[2] = pData[11];
					return_data[0] = STM32_WRITE_CMD;
					DEBUG_AnonyWriteReturn(return_data);
				}
				/*读取数据ID */
				switch (Debug_data_pack.PAR_ID)
				{
					case 0:test[0]=Debug_data_pack.PAR_VAL;break;
					case 1:test[1]=Debug_data_pack.PAR_VAL;break;
					case 2:test[2]=Debug_data_pack.PAR_VAL;break;
					default:break;
				}
			};break;
		/*数据读取指令 修改对应ID的数据*/
		case STM32_READ_CMD:
		{
					for (i=0;i<pData[3]+4;i++)
				{
					sumcheck += pData[i];
					addcheck += sumcheck;
				}
				if(sumcheck == pData[6]&&addcheck == pData[7])
				{
					memcpy(&PAR_ID,&pData[4],2);	
					switch (PAR_ID)
					{
						case 0:PAR_VAL=(int32_t)test[0];break;
						case 1:PAR_VAL=(int32_t)test[1];break;
						case 2:PAR_VAL=(int32_t)test[2];break;
						case 1000:PAR_VAL=0x11223344;break;
						default:break;
					}
					DEBUG_AnonyReadReturn(PAR_ID,PAR_VAL);
				}
		};break;
		default:break;
	}
}

/**
  * @brief         	ANO数传的串口中断函数 处理接收的数据
  * @param[in]      none
  * @retval         none
	*	@attention			空闲中断
  */

void USART1_IRQHandler(void)
{
    if(USART1->SR & UART_FLAG_IDLE)
    {
			//static uint16_t this_time_rx_len=0;
			__HAL_UART_CLEAR_PEFLAG(&huart1);

			if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
			{
				/* Current memory buffer used is Memory 0 */

				//disable DMA
				//失效DMA
				__HAL_DMA_DISABLE(&hdma_usart1_rx);

				//get receive data length, length = set_data_length - remain_length
				//获取接收数据长度,长度 = 设定长度 - 剩余长度
				//this_time_rx_len = DEBUG_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
				
				
				//reset set_data_lenght
				//重新设定数据长度
				hdma_usart1_rx.Instance->NDTR = DEBUG_RX_BUF_NUM;

				//set memory buffer 1
				//设定缓冲区1
				hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
				
				
				//enable DMA
				//使能DMA
				__HAL_DMA_ENABLE(&hdma_usart1_rx);

				//处理数传数据
				if (ANO_rx_buffer[0][0]==ANONY_HEAD)
					DEBUG_Data_solve(ANO_rx_buffer[0]);
				memset(ANO_rx_buffer[0],0,DEBUG_RX_BUF_NUM);
				if (ANO_rx_buffer[1][0]==ANONY_HEAD)
					 DEBUG_Data_solve(ANO_rx_buffer[1]);

			 }
			else
			{
				/* Current memory buffer used is Memory 1 */
				//disable DMA
				//失效DMA
				__HAL_DMA_DISABLE(&hdma_usart1_rx);

				//get receive data length, length = set_data_length - remain_length
				//获取接收数据长度,长度 = 设定长度 - 剩余长度
				//this_time_rx_len = DEBUG_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

				//reset set_data_lenght
				//重新设定数据长度
				hdma_usart1_rx.Instance->NDTR = DEBUG_RX_BUF_NUM;

				
				//set memory buffer 0
				//设定缓冲区0
				hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
				
				//enable DMA
				//使能DMA
				__HAL_DMA_ENABLE(&hdma_usart1_rx);
			
				 //处理数传数据
				 if (ANO_rx_buffer[1][0]==ANONY_HEAD)
					DEBUG_Data_solve(ANO_rx_buffer[1]);
				 memset(ANO_rx_buffer[1],0,DEBUG_RX_BUF_NUM);
				 if (ANO_rx_buffer[0][0]==ANONY_HEAD)
					 DEBUG_Data_solve(ANO_rx_buffer[0]);
			 }
    }
}
/**************************************************************************/
