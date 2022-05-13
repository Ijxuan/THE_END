/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//extern uint32_t time_every100us;
extern int time_every1s;
extern uint32_t time_every100ms;

//extern	uint32_t task2_times;
//extern uint32_t task3_times;
//extern uint32_t pendsv_times;
//extern uint32_t sysclk_to_pendsv;
//extern int guance1;
//extern int guance2;

#define HWswitch_1_Pin GPIO_PIN_8
#define HWswitch_2_Pin GPIO_PIN_9

extern int mydelay_100us;
extern uint32_t time7_times;//��ʱ��7�жϽ������

extern uint32_t task_init_times;
extern uint32_t task_can_times;
extern uint32_t task_can2_times;
extern uint32_t task_debug_times;
extern uint32_t task_controul_times;//�ܿ����������д���
extern int every_1s_times;//�������

extern float yaw_trage_angle;
extern float yaw_trage_angle2;
extern float yaw_trage_speed;
extern float PITCH_trage_angle;
extern float PITCH_trage_angle_2;

extern int PITCH_trage_speed;

extern int send_to_yaw;
extern int send_to_pitch;

extern float PITCH_IMU_Kp;//���ֵҪȡ��
extern float PITCH_MAX_angle;
extern float PITCH_MIN_angle;
extern int send_to_chassis;//���͸����̵�����
extern int CHASSIS_trage_speed;//���̵�Ŀ���ٶ�
extern int CHASSIS_trage_speed_last;//��һ�ε��̵�Ŀ���ٶ�

extern int CHASSIS_trage_angle;//���̵�Ŀ��Ƕ�
extern int send_to_SHOOT_R;//���͸��ҷ���Ħ���ֵ�����
extern int send_to_SHOOT_L;//���͸�����Ħ���ֵ�����
extern int send_to_driver;//���͸����̵��������

extern int CHASSIS_MAX_SPEED;//��������ٶ�

extern int M_3508_error;

extern int SHOOT_L;//��Ħ���ֵ�Ŀ���ٶ�
extern int SHOOT_R;//��Ħ���ֵ�Ŀ���ٶ�

extern bool HWswitch_R;
extern bool HWswitch_L;

extern bool HWswitch_R_last;
extern bool HWswitch_L_last;

extern int CHASSIS_L_MAX;
extern int CHASSIS_R_MIN;
extern int speed_change;
extern int CHASSIS_MID;
extern float DEBUFF;
extern int ENCODER_L_MAX;
extern int ENCODER_R_MIN;
extern int ENCODER_M_MID;
extern int ENCODER_LONG;//������ͨ�������µĹ������
extern int M3508_3ms_ago;//3������ǰ��ֵ
extern int M3508_3ms_change;//3����ı��ֵ
extern short int speed_change_times;
extern int ENCODER_ADD;//
extern float encoder_fbl_k;//�ֱ��ʵı�����ϵ
extern int ENCODER_SPEED;//��������ε�ֵ���ϴε�ֵ�õ����ٶ�
extern int M3508_3ms_ago_total_angle;//3������ǰ��ֵ
extern int M3508_3ms_ago_speed;//3����ı��ֵ
extern float M3508_speed_angle_kp;//�Ƕ����ٶȵĹ�ϵ
extern int ENCODER_ARRIVE_MAX;//�������ִ�����ֵ
extern int ENCODER_ARRIVE_MIN;//�������ִ����Сֵ

extern int32_t CHASSIS_L_MAX_by_ENCODER;//�����õ���߽�ֵ
extern int32_t CHASSIS_R_MIN_by_ENCODER;//�����õ��ұ߽�ֵ
extern int reverse_by_ENCODER; //������ǰֵ

extern int driver_targe_speed;//���̵�Ŀ���ٶ�


extern int uart_8_times;

extern int time_3_times;

extern float Vision_RawData_Yaw_Angle;
extern float Vision_RawData_Pitch_Angle;
extern bool send_to_C;
extern bool send_to_C_JS_SHOOT;
extern bool send_to_C_JS_HURT;//����ϵͳ_�˺�����_�Ƿ��͸�C��
extern bool send_to_C_JS_STATUS;//����ϵͳ_״̬����_�Ƿ��͸�C��
extern bool send_to_C_JS_HEAT;//����ϵͳ_��������_�Ƿ��͸�C��

extern bool send_to_C_IN_END;//�Ƿ� ���ڹ����ͷ_״̬����_�Ƿ��͸�C��
extern bool send_to_C_IN_MID;// ���ڹ���м�_״̬����_�Ƿ��͸�C��  1��һ��


extern bool in_MID;//���ڹ���м��
extern bool in_END;//���ڹ����ͷ
extern bool last_in_MID;//�ոմ��ڹ���м��
extern bool last_in_END;//�ոմ��ڹ����ͷ
extern int send_to_C_STATUS_times;//��Ϊ״̬���ݷ�4�η���,���Լ�������һ��

extern int JS_SEND_times;

extern int send_to_C_times;

extern int CH_TOTAL;

extern float Chassis_PowerLimit;
extern bool stop_chassic_output;
extern bool stop_CH_OP_BC_END;
extern int stop_CH_OP_BC_END_times;
extern bool stop_CH_OP_BC_LESS;
extern int stop_CH_OP_BC_LESS_times;
extern int CHASSIS_trage_speed_temp;


extern int uart_3_times;

extern int M3508_acceleration_cycle;//���ٶȼ�������(ÿ�μ���ļ��)(ms)
extern int M3508_last_speed;//��һʱ�̵��ٶ�,���ڼ��ٶȼ���
extern float M3508_acceleration; //���ٶ�

extern int hurt_times_ago;//���ǰ������
extern float KB_add_speed;//��ģʽ����
extern float KB_low_JB;//��ģʽ���幦������
extern float KB_high_JB;//��ģʽ���幦�ʻظ�Ŀ��

extern int usart_7_times;

extern bool disable_for_test;//Ϊ�˵���

extern bool state_Infrared_R_is_ok;//�ұߺ��⴫������ת����
extern bool state_Infrared_L_is_ok;//��ߺ��⴫������ת����
typedef struct
{
	bool step_1;
	bool step_2;
	bool step_3;
	bool step_4;
	bool step_5;

	
	
}JC;
extern int ch4_DW_total;

extern int send_to_chassis_special;//���͸����̵�����
extern bool use_special_send;
extern int ENCODER_CHANGE;//BMQ�ı�ֵ
extern int restart_times;//����ʱ��

extern int CAN2_SEND_TASK_times;//��������ʱ��
extern int place_SEND_times;//����λ�÷��ʹ���

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//��������Ĳ���
//��ջ��С
#define RobotCtrl_Size 512
//���ȼ�
#define RobotCtrl_Priority osPriorityRealtime

#define Sensor_Size 128

#define Sensor_Priority osPriorityHigh

//������
//extern TaskHandle_t RobotCtrl_Handle;
////�����������
//extern void Robot_Control(void const * argument);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int my_ms_delay1(uint32_t ms_Delay);
void my_ms_delay2(uint32_t ms_Delay2);
uint32_t MY_GetTick(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_A_Pin GPIO_PIN_0
#define Encoder_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_1
#define Encoder_B_GPIO_Port GPIOA
#define Judge_TX_Pin GPIO_PIN_8
#define Judge_TX_GPIO_Port GPIOD
#define Judge_RX_Pin GPIO_PIN_9
#define Judge_RX_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
