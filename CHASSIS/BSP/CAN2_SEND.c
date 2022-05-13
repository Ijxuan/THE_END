#include "CAN2_SEND.h"

uint8_t js_SEND_all[33];  // 4x8=32

void JS_send_SHOOT_control()
{
	for(int i=0;i<7;i++ )//0��6λ��Ч,һ��7λ
{
	js_SEND_all[i]=ext_shoot_data.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_SHOOT_ID,&js_SEND_all[0]);

}
void JS_send_HURT_control()
{

	js_SEND_all[0]=ext_robot_hurt.data.dataBuff[0];

	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_HURT_ID,&js_SEND_all[0]);

}

void JS_send_STATUS_control()
{

	for(int i=0;i<27;i++ )//0��26λ��Ч,һ��27λ
{
	js_SEND_all[i]=ext_game_robot_state.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_ONE,&js_SEND_all[0]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_TWO,&js_SEND_all[8]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_THREE,&js_SEND_all[16]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_FOUR,&js_SEND_all[24]);
osDelay(1);

}

void JS_send_HEAT_control()
{

	for(int i=0;i<18;i++ )//0��17λ��Ч,һ��18λ
{
	js_SEND_all[i]=ext_power_heat_data.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_HEAT_ID_ONE,&js_SEND_all[0]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_HEAT_ID_TWO,&js_SEND_all[8]);

}

void PLACE_send_control()
{

if(in_END==1)//���ڹ���м��,��ĩ��
{
	js_SEND_all[0]=1;
}
else//�ڹ���м��
{
	js_SEND_all[0]=0;
}

	js_SEND_all[1]=0;
	js_SEND_all[2]=0;

if(in_MID==1)//�ڹ���м��
{
	js_SEND_all[3]=1;
	js_SEND_all[4]=1;
}
else//���ڹ���м��
{
	js_SEND_all[3]=0;
	js_SEND_all[4]=0;	
}

	js_SEND_all[5]=0;
	js_SEND_all[6]=0;

if(in_END==1)//���ڹ���м��,��ĩ��
{
	js_SEND_all[7]=1;
}
else
{
	js_SEND_all[7]=0;
}
	
	
	CAN_SendData(&hcan2,CAN_ID_STD,PLACE_SEND_ID,&js_SEND_all[0]);


}

