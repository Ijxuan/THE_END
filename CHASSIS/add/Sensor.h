#ifndef SENSOR_H
#define SENSOR_H

//#include "PID_Position.h"
//#include "PID_Increment.h"
#include <stdio.h>
//#include "CRC.h"
#include "usart.h"
//#include "Robots_Detection.h"
//#include "User_typedef.h"
#include "CRC.h" 

#pragma anon_unions


#define SENSOR_BuffSize (9 + 2) //�Ӿ��������ݻ���������
extern uint8_t SENSOR_L_DataBuff[SENSOR_BuffSize];
extern uint8_t SENSOR_R_DataBuff[SENSOR_BuffSize];

typedef struct
{
	struct
	{
		union
		{
			struct
			{
				uint8_t Start_Tag_one; //֡ͷ
				uint8_t Start_Tag_two; //֡ͷ
				
				uint8_t DIST_L; //֡ͷ
				uint8_t DIST_H; //֡ͷ
				
				uint8_t APM_L; //֡ͷ
				uint8_t APM_H; //֡ͷ

				uint8_t TEMP_L; //֡ͷ
				uint8_t TEMP_H; //֡ͷ
				
				uint8_t ZHECK_SUM; //֡β


			};
			uint8_t SENSOR_RawData[9];
		};

		int DIST;				//Yaw��ĽǶ�
		int APM;				//Pitch��ĽǶ�
		int TEMP;					//���
	
	} RawData; //�Ӿ���Э�� ����һ֡�����ݽṹ��

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//���ݴ����־λ
	uint8_t DataUpdate_Flag;				//���ݸ��±�־λ
	


} sensor_t;

void Sensor_R_DataReceive(uint8_t *data);
void SENSOR_Handler(UART_HandleTypeDef *huart);

void Sensor_L_DataReceive(uint8_t *data);
void SENSOL_Handler(UART_HandleTypeDef *huart);

extern sensor_t Sensor_L;
extern sensor_t Sensor_R;
#endif