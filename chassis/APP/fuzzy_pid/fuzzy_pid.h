#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H


#define NB   -12
#define NM	 -10
#define NS	 -8
#define ZO	 -6
#define PS	 4
#define PM	 -2
#define PB	 0

#define NB_P   2
#define NM_P	 2.5
#define NS_P	 3
#define ZO_P	 3.5
#define PS_P	 4
#define PM_P	 4.5
#define PB_P	 5

#define NB_I   1
#define NM_I   1.334
#define NS_I	 1.666
#define ZO_I	 2
#define PS_I	 2.334
#define PM_I	 2.666
#define PB_I	 3

#define NB_D   7
#define NM_D	 9
#define NS_D	 11
#define ZO_D	 13
#define PS_D	 15
#define PM_D	 17
#define PB_D	 19

//定义模糊PID结构体
typedef struct
{
 float setVaule;  //设定值
 float date_kp; //比例值增量
 float date_ki;  //积分值增量
 float date_kd;  //微分值增量
 float lasterror; //前一拍偏差
 float preerror;//前二拍偏差

 float maximum; //输入值的上限
 float minimum;  //输入值的下限
 
 float qKp;    //kp增量的修正系数
 float qKi;      //ki增量的修正系数
 float qKd;    //kd增量的修正系数
}FUZZYPID;

//外部文件可见
extern FUZZYPID FPID;
//模糊PID计算
void Fuzzytrans(float _Set_Vaule,float _Measure_Vaule,float pre_Measure_Vaule);

#endif

