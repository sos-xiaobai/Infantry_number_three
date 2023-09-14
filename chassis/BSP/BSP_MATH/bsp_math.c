#include "bsp_math.h"

int abs(int a)
{
	if(a<0) return -a;
	else return a;
}

float fabss(float a)
{
	if(a<0.0f) return -a;
	else return a;
}
//大小范围限制
uint16_t limits_(uint16_t maxx,uint16_t minn,uint16_t a)
{
	if(a>maxx) return maxx;
	else if(a<minn) return minn;
	else return a;
}
//范围转换
float limits_change(int maxx,int minn,int a,int maxx_actual,int minn_actual)
{
	float b=(float)(a-minn_actual)/(float)(maxx_actual-minn_actual);
	float c=(float)b*(float)(maxx-minn);
	float ans=c+(float)minn;
	return ans;
}
