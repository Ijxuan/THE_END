#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"

//CAN数据队列句柄
extern QueueHandle_t CAN1_Queue;
extern QueueHandle_t CAN2_Queue;

typedef struct
{
	CAN_TxHeaderTypeDef CAN_TxMessage;				//CAN 发送缓冲区句柄
	uint8_t CAN_TxMessageData[8];				//存储CAN发送数据 数组
}CAN_Tx_Typedef;

//没必要，别的文件没有要用到，而且用作检测时，才为全局
//extern CAN_Tx_Typedef CAN_Tx_Structure;				//CAN发送结构体 

typedef struct
{
//	uint8_t CANx;															//指定哪个CAN
	CAN_RxHeaderTypeDef CAN_RxMessage;				//CAN 接收缓冲区句柄
	uint8_t CAN_RxMessageData[8];				//存储CAN接收数据 数组
}CAN_Rx_TypeDef;

//没有必要将其作为全局变量,只有在调试检测要观测变量时，才为全局
//extern CAN_Rx_TypeDef CAN1_Rx_Structure;				//CAN1接收结构体
//extern CAN_Rx_TypeDef CAN2_Rx_Structure;				//CAN2接收结构体

void CAN1_Filter0_Init(void);
void CAN1_Config(void);

void CAN2_Filter0_Init(void);
void CAN2_Config(void);

void CAN_SendData(CAN_HandleTypeDef* hcanx,uint8_t id_type,uint32_t id,uint8_t data[8]);

#endif