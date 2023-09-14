/**
  ******************************************************************************
  * @file    oled.c
  * @author  ZWT
  * @version V1.0
  * @date    2021-xx-xx
  * @brief   中景园0.96寸OLED屏外设
	*					 使用IIC模式，显示屏地址0x78,一次发送两个字节 0x00命令 0x40数据 控制数据帧
	*					 address 	mode 				data
  *			     0x78    	0x40/0x00		0xXX
  ******************************************************************************
  * @attention
  * 平台:		RoboMaster C型开发板 STM32F407IGHx
  * QQ：		2457513659
  * Email:	zwt190315@163.com
  *	@reference 
	*		DJI C型开发板例程
	*		中景园电子OLED屏例程
  * @function
  *		uint8_t OLED_SendByte(uint8_t dat, uint8_t cmd)
  * @parament
  *		
  *	
  ******************************************************************************
**/ 

#include "oled.h"
#include "bsp_i2c.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

extern I2C_HandleTypeDef hi2c2;
OLED_GRAM_strutct_t oled_gram;

void OLED_com_reset(void)
{
    static uint16_t time = 0;
    time++;
    if(time > 100)
    {
        bsp_I2C_reset(OLED_I2C);
        time = 0;
    }
}



/**
 * @brief   write data/command to OLED, if you use spi, please rewrite the function
 * @param   dat: the data ready to write
 * @param   cmd: OLED_CMD means command; OLED_DATA means data
 * @retval  none
 */
 void OLED_SendByte(uint8_t dat, uint8_t cmd)
{
    static uint8_t cmd_data[2];
    if(cmd == OLED_CMD)
    {
        cmd_data[0] = 0x00;
    }
    else
    {
        cmd_data[0] = 0x40;
    }
    cmd_data[1] = dat;
    bsp_I2C_master_transmit(OLED_I2C, OLED_I2C_ADDRESS, cmd_data, 2);
}


/**
  * @brief  Basic management of the timeout situation.
  * @param  errorCode：错误代码，可以用来定位是哪个环节出错.
  * @retval 返回0，表示IIC读取失败.
  */
void OLED_init(void)
{
		I2C2_tx_DMA_init();
	
    OLED_SendByte(0xAE, OLED_CMD);
    OLED_SendByte(0x20, OLED_CMD);
    OLED_SendByte(0x00, OLED_CMD);
    OLED_SendByte(0x21, OLED_CMD);
    OLED_SendByte(0x00, OLED_CMD);
    OLED_SendByte(0x7F, OLED_CMD);
    OLED_SendByte(0x22, OLED_CMD);
    OLED_SendByte(0x00, OLED_CMD);
    OLED_SendByte(0x07, OLED_CMD);
    OLED_SendByte(0x3F, OLED_CMD);
    OLED_SendByte(0x81, OLED_CMD);
    OLED_SendByte(0xFF, OLED_CMD);
    OLED_SendByte(0xA1, OLED_CMD);
    OLED_SendByte(0xA6, OLED_CMD);
    OLED_SendByte(0xA8, OLED_CMD);
    OLED_SendByte(0x3F, OLED_CMD);
    OLED_SendByte(0xC8, OLED_CMD);
    OLED_SendByte(0xD3, OLED_CMD);
    OLED_SendByte(0x00, OLED_CMD);
    OLED_SendByte(0xD5, OLED_CMD);
    OLED_SendByte(0x80, OLED_CMD);
    OLED_SendByte(0xD9, OLED_CMD);
    OLED_SendByte(0x1F, OLED_CMD);
    OLED_SendByte(0xDA, OLED_CMD);
    OLED_SendByte(0x12, OLED_CMD);
    OLED_SendByte(0xDB, OLED_CMD);
    OLED_SendByte(0x30, OLED_CMD);
    OLED_SendByte(0x8d, OLED_CMD);
    OLED_SendByte(0x14, OLED_CMD);
    OLED_SendByte(0xAF, OLED_CMD);
}

/**
  * @brief          turn on OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          打开OLED显示
  * @param[in]      none
  * @retval         none
  */
void OLED_display_on(void)
{
    OLED_SendByte(0x8d, OLED_CMD);
    OLED_SendByte(0x14, OLED_CMD);
    OLED_SendByte(0xaf, OLED_CMD);
}

/**
  * @brief          turn off OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          关闭OLED显示
  * @param[in]      none
  * @retval         none
  */
void OLED_display_off(void)
{
    OLED_SendByte(0x8d, OLED_CMD);
    OLED_SendByte(0x10, OLED_CMD);
    OLED_SendByte(0xae, OLED_CMD);
}

/**
  * @brief          turn OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          翻转OLED显示
  * @param[in]      none
  * @retval         none
  */
uint8_t Look1=0,Look2=0;
void OLED_display_turn(void)
{
	static uint8_t flag=0;
		if(flag==0)
		{
			OLED_SendByte(0xC8,OLED_CMD);//正常显示
			OLED_SendByte(0xA1,OLED_CMD);
			flag=1;
		}
	else
		{
			OLED_SendByte(0xC0,OLED_CMD);//反转显示
			OLED_SendByte(0xA0,OLED_CMD);
			flag=0;
		}
}

/**
  * @brief          claer OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          清除OLED显示
  * @param[in]      none
  * @retval         none
  */
void OLED_display_claer(void)
{
	memset(oled_gram.OLED_GRAM, 0, 1024);
	OLED_refresh_gram();
}
/**
  * @brief          operate the graphic ram(size: 128*8 char)
  * @param[in]      pen: the type of operate.
                    PEN_CLEAR: set ram to 0x00
                    PEN_WRITE: set ram to 0xff
                    PEN_INVERSION: bit inversion 
  * @retval         none
  */
/**
  * @brief          操作GRAM内存(128*8char数组)
  * @param[in]      pen: 操作类型.
                    PEN_CLEAR: 设置为0x00
                    PEN_WRITE: 设置为0xff
                    PEN_INVERSION: 按位取反
  * @retval         none
  */
/**
 * @brief   operate the graphic ram(size: 128*8 char)
 * @param   pen: the type of operate.
            PEN_CLEAR: set ram to 0x00
            PEN_WRITE: set ram to 0xff
            PEN_INVERSION: bit inversion 
 * @retval  none
 */
void OLED_operate_gram(pen_typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == PEN_WRITE)
            {
                oled_gram.OLED_GRAM[i][n] = 0xff;
            }
            else if (pen == PEN_CLEAR)
            {
                oled_gram.OLED_GRAM[i][n] = 0x00;
            }
            else
            {
                oled_gram.OLED_GRAM[i][n] = 0xff - oled_gram.OLED_GRAM[i][n];
            }
        }
    }
}

/**
  * @brief          cursor set to (x,y) point
  * @param[in]      x:X-axis, from 0 to 127
  * @param[in]      y:Y-axis, from 0 to 7
  * @retval         none
  */
/**
  * @brief          设置光标起点(x,y)
  * @param[in]      x:x轴, 从 0 到 127
  * @param[in]      y:y轴, 从 0 到 7
  * @retval         none
  */
void OLED_set_pos(uint8_t x, uint8_t y)
{
    x &= 0x7F;
    y &= 0x07;

    OLED_SendByte(0x21, OLED_CMD);
    OLED_SendByte(0x00 + x, OLED_CMD);
    OLED_SendByte(0x7F, OLED_CMD);

    OLED_SendByte(0x22, OLED_CMD);
    OLED_SendByte(0x00 + y, OLED_CMD);
    OLED_SendByte(0x07, OLED_CMD);
}

/**
  * @brief          draw one bit of graphic raw, operate one point of screan(128*64)
  * @param[in]      x: x-axis, [0, X_WIDTH-1]
  * @param[in]      y: y-axis, [0, Y_WIDTH-1]
  * @param[in]      pen: type of operation,
                        PEN_CLEAR: set (x,y) to 0
                        PEN_WRITE: set (x,y) to 1
                        PEN_INVERSION: (x,y) value inversion 
  * @retval         none
  */
/**
  * @brief          操作GRAM中的一个位，相当于操作屏幕的一个点
  * @param[in]      x:x轴,  [0,X_WIDTH-1]
  * @param[in]      y:y轴,  [0,Y_WIDTH-1]
  * @param[in]      pen: 操作类型,
                        PEN_CLEAR: 设置 (x,y) 点为 0
                        PEN_WRITE: 设置 (x,y) 点为 1
                        PEN_INVERSION: (x,y) 值反转
  * @retval         none
  */
void OLED_draw_point(uint8_t x, uint8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ( (x > (X_WIDTH - 1)) || (y > (Y_WIDTH - 1)))
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        oled_gram.OLED_GRAM[page][x] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        oled_gram.OLED_GRAM[page][x] ^= 1 << row;
    }
    else
    {
        oled_gram.OLED_GRAM[page][x] &= ~(1 << row);
    }
}

/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_draw_point(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}

/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
void OLED_show_char(uint8_t col, uint8_t row, char chr)
{
    uint8_t x = col;
    uint8_t y = row ;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}

/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
void OLED_show_string(uint8_t col, uint8_t row, char *chr)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_show_char(col, row, chr[n]);
        col+=6;

        if (col > X_WIDTH - 6)
        {
            col = 0;
            row += 12;
        }
        n++;
    }
}

/**
 * @brief   formatted output in oled 128*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 0 <= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
void OLED_printf(uint8_t col, uint8_t row, const char *fmt,...)
{
    static char LCD_BUF[22] = {0};
    static va_list ap;
    uint16_t remain_size = 0;

    va_start(ap, fmt);

    remain_size = vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);



    LCD_BUF[remain_size] = '\0';

    OLED_show_string(col, row, LCD_BUF);
}

/**
 * @brief   send the data of gram to oled sreen
 * @param   none
 * @retval  none
 */
void OLED_refresh_gram(void)
{
    OLED_set_pos(0, 0);
    oled_gram.cmd_data = 0x40;
    //I2C2_DMA_transmit(OLED_I2C_ADDRESS, (uint8_t*)&oled_gram, 1025);
		//DMA传输无法实现 参考DJI官方步兵代码 TODO
		bsp_I2C_master_transmit(OLED_I2C, OLED_I2C_ADDRESS, (uint8_t*)&oled_gram, 1025);
}

/**
 * @brief   show the logo of robomaster
 * @param   none
 * @retval  none
 */
void OLED_LOGO(void)
{
    memcpy(oled_gram.OLED_GRAM, RM_LOGO_BMP_TRANS, 1024);
    //OLED_show_graphic(0, 0, &rm_logo);
    OLED_refresh_gram();
}

void OLED_show_graphic(uint8_t x, uint8_t y, const picture_t *graphic)
{
    uint8_t col, row;
    uint8_t temp_char, t;
    uint16_t i = 0;

    for(col = 0; col < graphic->length; col++)
    {
        for(row = 0; row < graphic->width; )
        {
            temp_char = graphic->data[i];
            i++;
            for(t = 0; t < 8; t++)
            {
                if(temp_char & 0x80)
                {
                    OLED_draw_point(x + col, y + row,PEN_WRITE);
                }
                else
                {
                    OLED_draw_point(x + col, y + row,PEN_CLEAR);
                }
                temp_char <<= 1;
                row++;
                if(row == graphic->width)
                {
                    break;
                }
            }
        }
    }
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  errorCode：错误代码，可以用来定位是哪个环节出错.
  * @retval 返回0，表示IIC读取失败.
  */
//static  uint8_t I2C_TIMEOUT_UserCallback(uint8_t errorCode)
//{
//  /* Block communication and all processes */
//  //EEPROM_ERROR("I2C 等待超时!errorCode = %d",errorCode);
//  
//  return 0;
//}
/*********************************************END OF FILE**********************/
