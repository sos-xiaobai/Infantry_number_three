#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>

#define DMA_REC_LEN    100



#define Super_Cap_RX_Typecode 			12
#define SuperCap_Power_TX_Typecode		13
#define SuperCap_KeepAlive_TX_Typecode	14
#define SuperCap_Status_RX_Typecode		15
#define SuperCap_ErrorCode_RX_Typecode	41

extern uint8_t UART1_Sent_Data[10];
extern uint8_t UART1_Data_Type;
extern uint8_t dma_rx_buff[20];
extern void uart_init(void);
void DMA_Send(void);
#endif

