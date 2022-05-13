#ifndef __DR16RECIVE_H
#define __DR16RECIVE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>


void usart1_dr16_init(void);
void DR16_Process(uint8_t *pData);
void DR_16hander(UART_HandleTypeDef *huart);
void NM_swj(void);
void NM_swj2(void);

#define DR16BufferNumber 22
#define DR16BufferTruthNumber 18
#define DR16BufferLastNumber 4
//extern UART_HandleTypeDef huart1;

extern uint8_t DR16Buffer[DR16BufferNumber];
extern uint8_t CHASSIS_place[8];

extern uint8_t JSBuffer[8];

#define DR16_GroundInit \
{ \
{0,0,0,0,0,0,0}, \
{0,0,0,0,0}, \
{0}, \
0, \
0, \
&DR16_Process, \
} \


typedef struct
{
	struct {
		int16_t ch0;//yaw
		int16_t ch1;//pitch
		int16_t ch2;//left_right
		int16_t ch3;//forward_back
		uint8_t s_left;
		uint8_t s_right;
		int16_t ch4_DW; //����
	}rc;//ң�������յ���ԭʼֵ��

	struct {
		int16_t x;
		int16_t y;
		int16_t z;

		uint8_t keyLeft;
		uint8_t keyRight;

	}mouse;

	union {//union�������÷�
		uint16_t key_code;
		struct { //λ���ʹ��
			bool press_W : 1;
			bool press_S : 1;
			bool press_A : 1;
			bool press_D : 1;

			bool press_Shift : 1;
			bool press_Ctrl : 1;
			bool press_Q : 1;
			bool press_E : 1;

			bool press_R : 1;
			bool press_F : 1;
			bool press_G : 1;
			bool press_Z : 1;

			bool press_X : 1;
			bool press_C : 1;
			bool press_V : 1;
			bool press_B : 1;
		};
	}keyBoard;



	uint16_t infoUpdateFrame;	//֡��
	uint8_t offLineFlag;		//�豸���߱�־

	/*ָ�뺯��*/
//	void(*DR16_ReInit)(void);
	void(*DR16_Process)(uint8_t *pData);
//	void(*DR16_Handler)(UART_HandleTypeDef *huart);

} DR16_t;
extern DR16_t DR16;


/* ID: 0X0207          Byte: 7       ʵʱ������� */
typedef struct
{
  union
	{
		uint8_t dataBuff[7];
		__packed struct
		{
		  uint8_t bullet_type;//�ӵ�����: 1��17mm ���� 2��42mm ����
      uint8_t shooter_id;//������� ID
      uint8_t bullet_freq;//�ӵ���Ƶ ��λ Hz
      float bullet_speed;//�ӵ����� ��λ m/s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_shoot_data_t;
extern ext_shoot_data_t ext_shoot_data;

/* ID: 0X0206          Byte: 1       �˺�״̬���� */
typedef struct
{
  union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
		  uint8_t armor_id : 4;/*bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵�
                             ���װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0��*/
      uint8_t hurt_type : 4;/*bit 4-7��Ѫ���仯����,0x0 װ���˺���Ѫ��
                              0x1 ģ����߿�Ѫ��
                              0x2 �����ٿ�Ѫ��
                              0x3 ��ǹ��������Ѫ��
                              0x4 �����̹��ʿ�Ѫ��
                              0x5 װ��ײ����Ѫ*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_robot_hurt_t;
extern ext_robot_hurt_t ext_robot_hurt;

/* ID: 0X0201          Byte: 27      ������״̬���� */
typedef struct
{
	union
	{
		uint8_t dataBuff[27];
		__packed struct
		{
		  uint8_t robot_id;
      uint8_t robot_level;
      uint16_t remain_HP;//������ʣ��Ѫ��
      uint16_t max_HP;//����������Ѫ��
      uint16_t shooter_id1_17mm_cooling_rate; //������ 1 �� 17mm ǹ��ÿ����ȴֵ
      uint16_t shooter_id1_17mm_cooling_limit;//������ 1 �� 17mm ǹ����������
      uint16_t shooter_id1_17mm_speed_limit;  //������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
      uint16_t shooter_id2_17mm_cooling_rate;
      uint16_t shooter_id2_17mm_cooling_limit;
      uint16_t shooter_id2_17mm_speed_limit;
      uint16_t shooter_id1_42mm_cooling_rate;
      uint16_t shooter_id1_42mm_cooling_limit;
      uint16_t shooter_id1_42mm_speed_limit;
      uint16_t chassis_power_limit;           //�����˵��̹�����������
			/*���ص�Դ��������
       0 bit��gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
       1 bit��chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
       2 bit��shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����*/
      uint8_t mains_power_gimbal_output : 1;
      uint8_t mains_power_chassis_output : 1;
      uint8_t mains_power_shooter_output : 1;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_status_t;

extern ext_game_robot_status_t ext_game_robot_state;


/* ID: 0X0202          Byte: 16      ʵʱ������������ */
typedef struct
{
  union
	{
		uint8_t dataBuff[16];
		__packed struct
		{
			uint16_t chassis_volt; //���������ѹ ��λ ����
      uint16_t chassis_current; //����������� ��λ ����
      float chassis_power;//����������� ��λ W ��
      uint16_t chassis_power_buffer;//���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
      uint16_t shooter_id1_17mm_cooling_heat;//ǹ������
      uint16_t shooter_id2_17mm_cooling_heat;
      uint16_t shooter_id1_42mm_cooling_heat;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_power_heat_data_t;

extern ext_power_heat_data_t ext_power_heat_data;

/**********Ϊ������������λ����Э�鶨��ı���****************************/
//cupΪС��ģʽ�洢��Ҳ�����ڴ洢��ʱ�򣬵�λ������0�ֽڣ���λ��1�ֽ�
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))     //ȡ��int�ͱ����ĵ��ֽ�
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))     //    ȡ�洢�ڴ˱�����һ�ڴ��ֽڵ����ݣ����ֽ�
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#endif
