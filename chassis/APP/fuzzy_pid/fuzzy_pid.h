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

//����ģ��PID�ṹ��
typedef struct
{
 float setVaule;  //�趨ֵ
 float date_kp; //����ֵ����
 float date_ki;  //����ֵ����
 float date_kd;  //΢��ֵ����
 float lasterror; //ǰһ��ƫ��
 float preerror;//ǰ����ƫ��

 float maximum; //����ֵ������
 float minimum;  //����ֵ������
 
 float qKp;    //kp����������ϵ��
 float qKi;      //ki����������ϵ��
 float qKd;    //kd����������ϵ��
}FUZZYPID;

//�ⲿ�ļ��ɼ�
extern FUZZYPID FPID;
//ģ��PID����
void Fuzzytrans(float _Set_Vaule,float _Measure_Vaule,float pre_Measure_Vaule);

#endif

