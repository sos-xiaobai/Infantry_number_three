#ifndef __ANO_H
#define	__ANO_H

#include "struct_typedef.h"
#include "string.h"
#include "stdio.h"

/*上位机控制指令的最大字节长度*/
#define DEBUG_RX_BUF_NUM 12u
/***匿名数传通信格式 数据头0xAA 硬件地址0xAF 上位机****/
#define ANONY_HEAD	0xAA
#define ANONY_ADDR	0xAF
#define ANONY_ID1		0xF1	//数传灵活帧 ID范围从0xF1~0xFA
#define ANONY_ID2		0xF2
#define ANONY_ID3		0xF3
#define ANONY_ID4		0xF4
#define ANONY_ID5		0xF5
#define ANONY_ID6		0xF6
#define ANONY_ID7		0xF7
#define ANONY_ID8		0xF8
#define ANONY_ID9		0xF9
#define ANONY_IDA		0xFA
#define ANONY_LEN		40		//数传数据段长度 最长40
#define STM32_ADDR	0xFF	//下位机接收地址
#define PC_ADDR			0xAF		//上位机接收地址
#define STM32_WRITE_CMD 0xE2	//下位机写入,参数返回ID
#define STM32_READ_CMD	0xE1	//下位机读取ID
#define TMS32_LOG_CMD		0xA0	//上位机显示字符串
#define TMS32_LOGS_CMD		0xA1	//上位机显示字符串+数字
#define STM32_WRITE_LEN		6			//写入数长度
#define STM32_READ_LEN		6			//读取数长度
#define RETURN_ID	0x00		//返回帧ID
/**************************************
 *有空可以加上参数读取的代码
 *参数写入 dataweite	参数写入返回 writereturn
 *参数读取 dataread		参数读取返回 readreturn
 *LOG信息输出	字符串
 *LOG信息输出 数字+字符串
**/
#pragma pack(push, 1) 
typedef struct 
{
	uint16_t PAR_ID;
	int32_t PAR_VAL;
}Debug_receive;

typedef struct 
{
	uint16_t DEBUG_ID;
	float DEBUG_VAL;
}Debug_data;
#pragma pack(pop)

extern void ANO_usart_init(void);
extern void DEBUG_strings(const char *pdata,uint8_t color);
extern void DEBUG_printf(const char *pdata,int32_t VAL);
extern void DEBUG_Anony16Send(uint8_t id,int16_t *pst,uint8_t len);
extern void DEBUG_Anony32Send(uint8_t id,int32_t *pst,uint8_t len);


#endif /**__ANO_H**/
