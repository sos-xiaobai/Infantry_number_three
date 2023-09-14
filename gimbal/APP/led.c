#include "led.h"
#include "main.h"
/*
led[0]   	blue
led[1]		green
led[2]		red
led[3]		orange
*/
ledDataType led[4]={0};
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOH_CLK_ENABLE();
	
	led[0].GPIO = GPIOH;
	led[0].PIN = GPIO_PIN_10;
	
	led[1].GPIO = GPIOH;
	led[1].PIN = GPIO_PIN_11;
	
	led[2].GPIO = GPIOH;
	led[2].PIN = GPIO_PIN_12;
	
	led[3].GPIO = GPIOD;
	led[3].PIN = GPIO_PIN_0;
	
	GPIO_InitStruct.Pin = led[0].PIN|led[1].PIN|led[2].PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	
}

/*horse race light~~~~~*/
void led_run(uint8_t tick)
{
	static unsigned char time = 0;
	static unsigned char i = 0;
	++time;
	if(time>tick)
	{
		time = 0;
		if(i<5)
		{
			for(uint8_t j = 0;j<4;j++)
			{
				if(j<i)led_on(&led[j]);
				else led_off(&led[j]);
			}
		}
		else
		{
			for(uint8_t j = 0;j<4;j++)
			{
				if(j<(9-i))led_off(&led[j]);
				else led_on(&led[j]);
			}
		}
		i++;
		i = (i==10)?0:i;
	}	
}


/*b lin b lin~~~~~*/
void led_blin(uint8_t tick,ledDataType* led)
{
	led->tick++;
	if(led->tick==tick)
	{
		led->tick=0;
		led_sw(led);
	}	
}
void led_on(ledDataType* led)
{
	HAL_GPIO_WritePin(led->GPIO,led->PIN,GPIO_PIN_SET);
}
void led_off(ledDataType* led)
{
	HAL_GPIO_WritePin(led->GPIO,led->PIN,GPIO_PIN_RESET);
}
void led_sw(ledDataType* led)
{
	HAL_GPIO_TogglePin(led->GPIO,led->PIN);
}
