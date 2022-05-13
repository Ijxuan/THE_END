#include "BSP_CAN.h"

//��Ҫ�����ԣ��Ͱ�����������Ϊȫ��
//CAN_Tx_Typedef CAN_Tx_Structure;				//CAN�ķ��ͽṹ��

void CAN1_Filter0_Init(void)
{
	
	CAN_FilterTypeDef CANx_Filter;
	
	CANx_Filter.SlaveStartFilterBank = 0;				
	CANx_Filter.FilterBank = 0;				
	
	CANx_Filter.FilterMode = CAN_FILTERMODE_IDMASK;				
	CANx_Filter.FilterScale = CAN_FILTERSCALE_32BIT;				
	
	CANx_Filter.FilterIdHigh = 0x0000;				
	CANx_Filter.FilterIdLow = 0x0000;					
	CANx_Filter.FilterMaskIdHigh = 0x0000;					
	CANx_Filter.FilterMaskIdLow = 0x0000;
	
	CANx_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//ɸѡ����������FIFO0	
												 
	CANx_Filter.FilterActivation = ENABLE; //ʹ��ɸѡ��				
	
	
	if(HAL_CAN_ConfigFilter(&hcan1,&CANx_Filter) != HAL_OK)
	{
//		while(1)
//		{
//		}
	}
}





CAN_Tx_Typedef CAN_Tx_Structure;				
void CAN_SendData(CAN_HandleTypeDef* hcanx,uint8_t id_type,uint32_t id,uint8_t data[8])
{

	uint32_t Len;				
	uint8_t i;				
	uint32_t TxMailbox;				
	
	CAN_Tx_Structure.CAN_TxMessage.RTR = CAN_RTR_DATA;				
	CAN_Tx_Structure.CAN_TxMessage.IDE = id_type;				
	
	if(CAN_Tx_Structure.CAN_TxMessage.IDE == CAN_ID_STD)				
	{
		CAN_Tx_Structure.CAN_TxMessage.StdId = id;								
	}
	else if(CAN_Tx_Structure.CAN_TxMessage.IDE == CAN_ID_EXT)				
	{
		CAN_Tx_Structure.CAN_TxMessage.ExtId = id;
	}
	
	CAN_Tx_Structure.CAN_TxMessage.DLC = 0x08;				
	
	CAN_Tx_Structure.CAN_TxMessage.TransmitGlobalTime = DISABLE;				
	
	Len = CAN_Tx_Structure.CAN_TxMessage.DLC;
	for(i = 0 ; i < Len ; i++)
	{
		CAN_Tx_Structure.CAN_TxMessageData[i] = data[i];				
	}
	
	
	HAL_CAN_AddTxMessage(hcanx,&CAN_Tx_Structure.CAN_TxMessage,CAN_Tx_Structure.CAN_TxMessageData,&TxMailbox);
	
}


void CAN1_Config(void)
{
	
	CAN1_Filter0_Init();
	
	
//	if(HAL_CAN_Start(&hcan1)!= HAL_OK)
//	{
////		while(1)
////		{
////		}
//	}
	
		HAL_CAN_Start(&hcan1);

	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);         
//	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//	{
//		while(1)
//		{
//		}
//	}
}

void CAN2_Config(void)
{
	
	CAN2_Filter0_Init();
	

	if(HAL_CAN_Start(&hcan2)!= HAL_OK)
	{
		while(1)
		{
		}
	}
	
	
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);         
//	if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//	{
//		while(1)
//		{
//		}
//	}
}

void CAN2_Filter0_Init(void)
{

	CAN_FilterTypeDef CANx_Filter;
	
	CANx_Filter.SlaveStartFilterBank = 0;				
	CANx_Filter.FilterBank = 0;				
	
	CANx_Filter.FilterMode = CAN_FILTERMODE_IDMASK;				
	CANx_Filter.FilterScale = CAN_FILTERSCALE_32BIT;				
	
	CANx_Filter.FilterIdHigh = 0x0000;				
	CANx_Filter.FilterIdLow = 0x0000;					
	CANx_Filter.FilterMaskIdHigh = 0x0000;					
	CANx_Filter.FilterMaskIdLow = 0x0000;
	
	CANx_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;				
	
	CANx_Filter.FilterActivation = ENABLE;				
	
	//CAN_Filter ��ʼ������
	if(HAL_CAN_ConfigFilter(&hcan2,&CANx_Filter) != HAL_OK)
	{
		while(1)
		{
		}
	}
}

