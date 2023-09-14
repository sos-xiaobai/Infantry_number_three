#ifndef __REFEREE_USART_H
#define __REFEREE_USART_H

#include "main.h"
#include "usart.h"
#include "fifo.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024


extern fifo_s_t referee_fifo;

void referee_usart_fifo_init(void);
void RE_usart_tx_dma_enable(uint8_t *data, uint16_t len);

#endif
