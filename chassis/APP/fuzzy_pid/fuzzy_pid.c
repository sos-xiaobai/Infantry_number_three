#include "fuzzy_pid.h"

 //�����
static const float ruleKp[7][7]={
	PB_P,	PB_P,	PB_P,	PM_P,	PM_P,	PS_P,	ZO_P,
	PB_P,	PB_P,	PB_P,	PM_P,	PM_P,	PS_P,	ZO_P,
	PB_P,	PM_P,	PM_P,	ZO_P,	PS_P,	ZO_P,	PS_P,
	PM_P,	PS_P,	PS_P,	NS_P,	ZO_P,	NS_P,	NS_P,
	PS_P,	ZO_P,	ZO_P,	NS_P,	NS_P,	NM_P,	NM_P,
	ZO_P,	ZO_P,	NS_P,	NM_P,	NM_P,	NM_P,	NB_P,
	ZO_P,	NS_P,	NM_P,	NM_P,	NB_P,	NB_P,	NB_P
};
 
static const float ruleKi[7][7]={
	NB_I,	NB_I,	NM_I,	NM_I,	NS_I,	ZO_I,	ZO_I,
	NB_I,	NM_I,	NM_I,	NS_I,	NS_I,	ZO_I,	ZO_I,
	NM_I,	NM_I,	NS_I,	ZO_I,	ZO_I,	PS_I,	PS_I,
	NM_I,	NS_I,	ZO_I,	ZO_I,	PS_I,	PS_I,	PM_I,
	NS_I,	ZO_I,	ZO_I,	PS_I,	PS_I,	PM_I,	PM_I,
	ZO_I,	ZO_I,	PS_I,	PM_I,	PM_I,	PM_I,	PB_I,
	ZO_I,	ZO_I,	PM_I,	PM_I,	PB_I,	PB_I,	PB_I
};
 
static const float ruleKd[7][7]={
	PS_D,	NS_D,	NM_D,	NB_D,	NM_D,	NM_D,	PB_D,
	PS_D,	NS_D,	NS_D,	NM_D,	NS_D,	NM_D,	ZO_D,
	ZO_D,	NB_D,	ZO_D,	NS_D,	NS_D,	NS_D,	ZO_D,
	ZO_D,	ZO_D,	ZO_D,	ZO_D,	ZO_D,	ZO_D,	ZO_D,
	ZO_D,	PS_D,	PS_D,	PS_D,	ZO_D,	PS_D,	PM_D,
	ZO_D,	PS_D,	PS_D,	PM_D,	PS_D,	PS_D,	PB_D,
	ZO_D,	PS_D,	PM_D,	PB_D,	PS_D,	PM_D,	PB_D
};
 
//��ʼ���ṹ�����
FUZZYPID FPID={
	0,0,0,0,0,0,
	0,0,1,0,1
};

//�����ȼ��㺯��
static void CalcMembership(float *ms,float qv,int * index)
{
 if((qv>=NB)&&(qv<NM))
 {
 index[0]=0;
 index[1]=1;
 ms[0]=-0.5f*qv-5.0f;  //y=-0.5x-2.0
 ms[1]=0.5f*qv+6.0f;  //y=0.5x+3.0
 }
 else if((qv>=NM)&&(qv<NS))
 {
 index[0]=1;
 index[1]=2;
 ms[0]=-0.5f*qv-4.0f;  //y=-0.5x-1.0
 ms[1]=0.5f*qv+5.0f;  //y=0.5x+2.0
 }
 else if((qv>=NS)&&(qv<ZO))
 {
 index[0]=2;
 index[1]=3;
 ms[0]=-0.5f*qv-3.0f;  //y=-0.5x
 ms[1]=0.5f*qv+4.0f;  //y=0.5x+1.0
 }
 else if((qv>=ZO)&&(qv<PS))
 {
 index[0]=3;
 index[1]=4;
 ms[0]=-0.5f*qv-2.0f;  //y=-0.5x+1.0
 ms[1]=0.5f*qv+3.0f;  //y=0.5x
 }
 else if((qv>=PS)&&(qv<PM))
 {
 index[0]=4;
 index[1]=5;
 ms[0]=-0.5f*qv-1.0f;  //y=-0.5x+2.0
 ms[1]=0.5f*qv+2.0f;  //y=0.5x-1.0
 }
 else if((qv>=PM)&&(qv<=PB))
 {
 index[0]=5;
 index[1]=6;
 ms[0]=-0.5f*qv;  //y=-0.5x+3.0
 ms[1]=0.5f*qv+1.0f;  //y=0.5x-2.0
 }
}

//����ֵ����������(-6->6)
static void LinearQuantization(FUZZYPID *vPID,float _Real_Value,float *qValue)
{
  float thisError;
  float deltaError;

  thisError=vPID->setVaule-_Real_Value;   //���㵱ǰƫ��
  deltaError=thisError-vPID->lasterror;   //����ƫ������
	
  //E��EC������
  qValue[0]=6.0f*thisError/(vPID->maximum-vPID->minimum);
  qValue[1]=3.0f*deltaError/(vPID->maximum-vPID->minimum);

}

float __Real_Value;
//��ģ��
static void FuzzyComputation (FUZZYPID *vPID,float _Real_Value)
{
	//����ֵ
 float qValue[2]={0,0};  

 int indexE[2]={0,0};     //e�ڹ�����е�����
 float msE[2]={0,0};      //e��������
 
 int indexEC[2]={0,0};    //ec�ڹ�����е�����
 float msEC[2]={0,0};      //ec��������
 
 //pid����ֵ
 float pidvalue[3];
 
//����
 LinearQuantization(vPID,_Real_Value,qValue);
 __Real_Value=_Real_Value;
//����e�������Ⱥ�����
 CalcMembership(msE,qValue[0],indexE);
//����ec�������Ⱥ�����
 CalcMembership(msEC,qValue[1],indexEC);
 
 //�������ķ�����pid����ֵ
 pidvalue[0]=msE[0]*(msEC[0]*ruleKp[indexE[0]][indexEC[0]]+msEC[1]*ruleKp[indexE[0]][indexEC[1]]) 
            +msE[1]*(msEC[0]*ruleKp[indexE[1]][indexEC[0]]+msEC[1]*ruleKp[indexE[1]][indexEC[1]]);
 pidvalue[1]=msE[0]*(msEC[0]*ruleKi[indexE[0]][indexEC[0]]+msEC[1]*ruleKi[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKi[indexE[1]][indexEC[0]]+msEC[1]*ruleKi[indexE[1]][indexEC[1]]);
 pidvalue[2]=msE[0]*(msEC[0]*ruleKd[indexE[0]][indexEC[0]]+msEC[1]*ruleKd[indexE[0]][indexEC[1]])
            +msE[1]*(msEC[0]*ruleKd[indexE[1]][indexEC[0]]+msEC[1]*ruleKd[indexE[1]][indexEC[1]]);

//pid��������
 vPID->date_kp=vPID->qKp*pidvalue[0];
 vPID->date_ki=vPID->qKi*pidvalue[1];
 vPID->date_kd=vPID->qKd*pidvalue[2];

}

/**
  * @breif         ģ��pid����ں�����ʵ������ʱֻ����ô˺���
  * @param[in]     _Set_Vaule���趨��Ŀ��ֵ����target value
	* @param[in]     _Measure_Vaule�����ص���ʵֵ����actual value  
	* @param[in]     pre_Measure_Vaule����һ�η��ص���ʵֵ
	* @param[out]    vPID->date_kp��kp�������������Լ���д��pid��kp��
	* @param[out]    vPID->date_ki��ki�������������Լ���д��pid��ki��
	* @param[out]    vPID->date_kd��kd�������������Լ���д��pid��kd��
  * @retval        none     
  */

/*�������������*/
void Fuzzytrans(float _Set_Vaule,float _Measure_Vaule,float pre_Measure_Vaule) 
{	  
	//��һ�ε�ƫ��ֵ
	FPID.lasterror=_Set_Vaule-pre_Measure_Vaule;

	//�������������Сֵ
	FPID.maximum=50; 
	FPID.minimum=-50;
	FuzzyComputation(&FPID,_Measure_Vaule);
}

