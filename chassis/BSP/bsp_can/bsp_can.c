#include "bsp_can.h"

static int16_t* read_motor_data(uint8_t *rxdata);
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current);
static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current);
static void canTX_gimbal(int8_t robot_id, int16_t heat0, int16_t heat0_limit ,int16_t bullet_speed, uint8_t bullet_speedlimit);
int16_t gimbal_y_CAN_actual_speed=0;
int16_t gimbal_y_CAN_actual_angle=0;
uint8_t bullet_speedlimit=0;
uint8_t target_exit=0;
uint8_t run_away_flag=0;
uint8_t supercap_reboot_flag=0;
uint8_t reboot_flag=0;
int16_t vision_mode;

extern uint8_t chassis_power_flag;
/**
  * @breif         canͨ�ų�ʼ��
  * @param[in]     none
	* @param[out]    none
  * @retval        none     
  */
uint8_t bsp_can_init(void)
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	
	canFilter.FilterBank=1;    																//ɸѡ����1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//����ģʽ
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//����
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32λģʽ
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//���ӵ�fifo0
	canFilter.SlaveStartFilterBank=14;												//can2ɸѡ����ʼ���
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//���ù�����
	
	canFilter.FilterBank=15;    															//ɸѡ����15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//���ù�����
	
	/*�뿪��ʼģʽ*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	
	/*���ж�*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 ����fifo 0��Ϊ���ж�
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 ����fifo 0��Ϊ���ж�
	return status;
}


/**
  * @breif         can���ͺ���
  * @param[in]     hcan��can�ľ���ṹ��
	* @param[in]     mdata����Ҫ���͵�����
	* @param[out]    none
  * @retval        status�����͵�״̬    
  */
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata)
{
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	if(hcan==&hcan1)
	{
		CAN_TxHeaderStruct.StdId=0x200;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	else if(hcan==&hcan2)
	{
		CAN_TxHeaderStruct.StdId=0x006;
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	}
	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
	return status;
}

//����yaw��
uint8_t canTX_gimbal_y(int16_t yaw)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[6]=yaw>>8;
	data[7]=yaw&0xff;
	data[4]=0;
	data[5]=0;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}
int16_t spss;
/**
  * @breif         can�����жϺ���
  * @param[in]     hcan��can�ľ���ṹ��
	* @param[out]    can1�����ĸ�������صĵ���ֵ��can2�����ϰ崫��������
  * @retval        none   
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t rxdata[8];
	int16_t speed,*gdata,current;
	float angle;
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);
		gdata=read_motor_data(rxdata);
		angle=gdata[0];
		speed=gdata[1];
		current=gdata[2];
		switch(CAN_RxHeaderStruct.StdId)
		{
			case CAN_3508Motor1_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			break;
			case CAN_3508Motor2_ID:
				get_motor_data(&chassis_motor2,angle,speed,current);
			break;
			case CAN_3508Motor3_ID:
				get_motor_data(&chassis_motor3,angle,speed,current);
			break;
			case CAN_3508Motor4_ID:
				get_motor_data(&chassis_motor4,angle,speed,current);
			break;
			case CAN_GIMBAL_Y_ID:
				record_chassis_callback(&chassis_center,angle,current);
				gimbal_y_CAN_actual_speed=speed;
				gimbal_y_CAN_actual_angle=angle;
			break;
			case LOOP_BACK_ID:
				get_motor_data(&chassis_motor1,angle,speed,current);
			default:break;
		}
	}
	else if(hcan==&hcan2)
	{
		if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata)!=HAL_OK)
			chassis_control_order.chassis_mode=CHASSIS_NO_FORCE;
		gdata=read_motor_data(rxdata);
		if(CAN_RxHeaderStruct.StdId==GIMBAL_CONTROL_ID_1)
		{
			chassis_data_analyze();
			chassis_control_order.vx_set*=-1;
			chassis_control_order.vy_set*=-1;
		}
		if(CAN_RxHeaderStruct.StdId==GIMBAL_CONTROL_ID_2)	
		{
			chassis_control_order.last_chassis_mode=chassis_control_order.chassis_mode; 
			chassis_control_order.chassis_mode=gdata[0];
			vision_mode=gdata[1];
		target_exit=rxdata[4];
		supercap_reboot_flag=rxdata[5];
			run_away_flag=rxdata[6];
		reboot_flag=rxdata[7];

			
		}
	}
	
}

/**
  * @breif         can���ϵ�����ݺ���
  * @param[in]     rxdata��������ص�����
	* @param[out]    ���Ϻõ�����
  * @retval        adata�����Ϻõ�����  
  */
int16_t adata[4];
static int16_t* read_motor_data(uint8_t *rxdata)
{
		adata[0]=(int16_t)((rxdata[0]<<8)|rxdata[1]);
		adata[1]=(int16_t)((rxdata[2]<<8)|rxdata[3]);
		adata[2]=(int16_t)((rxdata[4]<<8)|rxdata[5]);
		adata[3]=(int16_t)((rxdata[6]<<8)|rxdata[7]);
	return adata;
}

/**
  * @breif         can����������ݺ���
  * @param[in]     motor����������ṹ��
  * @param[in]     angle������Ƕ�ֵ
  * @param[in]     speed������ٶ�ֵ
  * @param[in]     current���������ֵ
	* @param[out]    �������ֵ��ֵ������Ľṹ���Ա
  * @retval        none   
  */
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->pid.position_loop.apid.actual_angle=angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->pid.position_loop.vpid.actual_speed=motor->actual_speed;
	motor->pid.speed_loop.vpid.actual_speed=motor->actual_speed;
	//motor->actual_speed = KalmanFilter(speed,Q,R);
	motor->last_speed = speed;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//ֻ������ʱ��¼һ�γ�ʼ�Ƕ�
	}
	
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt--;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt++;
	motor->total_angle = motor->round_cnt * 8192 + motor->actual_angle;// - motor->start_angle;
}

static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = (float)angle/8191.0f;
	motor->actual_angle*=360.0f;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = (float)angle/8191.0f*360.0f;
		motor->start_angle_flag++;	//ֻ������ʱ��¼һ�γ�ʼ�Ƕ�
	}
	
}

void send_gimbal_data(void)  //���ϰ巢�Ͳ���ϵͳ����
{
	uint8_t id,bullet_freq,game_status;
	uint16_t heat0,heat0_limit;
	float bullet_speed;
	id=get_robot_id();
	get_shoot_heat0_limit_and_heat0(&heat0_limit,&heat0);
	get_shoot_data( &bullet_freq, &bullet_speed, &bullet_speedlimit);
	bullet_speed=10*bullet_speed;
  get_game_status(&game_status);
	canTX_gimbal(game_status, heat0, heat0_limit ,(int16_t)bullet_speed, bullet_speedlimit);
	
}

/**
  * @breif         can�����ϰ庯��
  * @param[in]     game_status���Ծ�״̬
  * @param[in]     heat0��ǹ������
  * @param[in]     heat0_limit��ǹ����������
  * @param[in]     bullet_speed���ӵ�����
  * @param[in]     bullet_speedlimit���ӵ���������
	* @param[out]    none
  * @retval        none   
  */
static void canTX_gimbal(int8_t game_status, int16_t heat0, int16_t heat0_limit ,int16_t bullet_speed, uint8_t bullet_speedlimit)
{
	uint8_t data[8]={0};
	
	data[0]=game_status;
	data[1]=heat0>>8;
	data[2]=heat0&0xff;
	data[3]=heat0_limit>>8;
	data[4]=heat0_limit&0xff;
	data[5]=bullet_speed>>8;
	data[6]=bullet_speed&0xff;
	data[7]=bullet_speedlimit;
	Can_Tx_Message(&hcan2,data);	
}

void send_gimbal_data_2(void)
{
	canTX_gimbal_2(gimbal_y_CAN_actual_speed,gimbal_y_CAN_actual_angle);
}

void canTX_gimbal_2(int16_t y_speed,int16_t y_angle)
{
	uint8_t data[8];
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=0x205;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=y_speed>>8;
	data[1]=y_speed&0xff;
	data[2]=y_angle>>8;
	data[3]=y_angle&0xff;
	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

	int16_t yaw_angle;
	int16_t yaw_speed;
	int16_t pitch_angle;
	int16_t pitch_speed;

void canTX_chassis_imu(void)
{
	uint8_t data[8];
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	yaw_angle=(int16_t)(chassis_center.yaw_angle*100);
	yaw_speed=(int16_t)(chassis_center.yaw_speed*100);
	pitch_angle=(int16_t)(chassis_center.pitch_angle*100);
	pitch_speed=(int16_t)(chassis_center.pitch_speed*100);
	
	CAN_TxHeaderStruct.StdId=0x012;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=yaw_angle>>8;
	data[1]=yaw_angle&0xff;
	data[2]=yaw_speed>>8;
	data[3]=yaw_speed&0xff;
	data[4]=pitch_angle>>8;
	data[5]=pitch_angle&0xff;
	data[6]=pitch_speed>>8;
	data[7]=pitch_speed&0xff;
	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}