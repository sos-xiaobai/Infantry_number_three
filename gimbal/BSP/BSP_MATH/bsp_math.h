#ifndef __bsp_math_H
#define __bsp_math_H

#include "stm32f4xx.h"

int abs(int a);
float fabss(float a);
uint16_t limits_(uint16_t maxx,uint16_t minn,uint16_t a);
float limits_change(int maxx,int minn,int a,int maxx_actual,int minn_actual);
int get_up_int_function(float num);
float limits_change_(int maxx,int minn,int a,int maxx_actual,int minn_actual,float left_range,float right_range);

#endif
