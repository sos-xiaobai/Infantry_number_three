#include "bsp_referee.h"

#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "referee.h"
#include "usart.h"

#define RE_huart  huart6
#define RE_UART		USART6
#define RE_dma_rx		hdma_usart6_rx
#define RE_dma_tx   hdma_usart6_tx


//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
static void REFEREE_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);





//主函数初始化调用
void referee_usart_fifo_init(void)
{
    REFEREE_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
		fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
}

static void REFEREE_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
		//enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(RE_huart.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(RE_huart.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&RE_huart, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&RE_dma_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&RE_dma_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&RE_dma_rx, DMA_LISR_TCIF2);

    RE_dma_rx.Instance->PAR = (uint32_t) & (RE_UART->DR);
    //memory buffer 1
    //内存缓冲区1
    RE_dma_rx.Instance->M0AR = (uint32_t)(usart6_buf);
    //memory buffer 2
    //内存缓冲区2
    RE_dma_rx.Instance->M1AR = (uint32_t)(usart6_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&RE_dma_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(RE_dma_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&RE_dma_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&RE_dma_tx);

    while(RE_dma_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&RE_dma_tx);
    }

    RE_dma_tx.Instance->PAR = (uint32_t) & (RE_UART->DR);
}


//写入stm32f4xx_it.c串口usart1中断
void USART6_IRQHandler(void)
{
	static volatile uint8_t res;
    if(RE_UART->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&RE_huart);

        static uint16_t this_time_rx_len = 0;

        if ((RE_huart.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(RE_huart.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(RE_huart.hdmarx);
            __HAL_DMA_SET_COUNTER(RE_huart.hdmarx, USART_RX_BUF_LENGHT);
            RE_huart.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(RE_huart.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(RE_huart.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(RE_huart.hdmarx);
            __HAL_DMA_SET_COUNTER(RE_huart.hdmarx, USART_RX_BUF_LENGHT);
            RE_huart.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(RE_huart.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
			}
}

/*串口发送*/
void RE_usart_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&RE_dma_tx);

    while(RE_dma_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&RE_dma_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&RE_dma_tx, DMA_HISR_TCIF7);

    RE_dma_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&RE_dma_tx, len);

    __HAL_DMA_ENABLE(&RE_dma_tx);
}


