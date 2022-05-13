/* USER CODE BEGIN Header */
/**       Code is far away from bug with the animal protecting
 *          .,:,,,                                        .::,,,::.
 *        .::::,,;;,                                  .,;;:,,....:i:
 *        :i,.::::,;i:.      ....,,:::::::::,....   .;i:,.  ......;i.
 *        :;..:::;::::i;,,:::;:,,,,,,,,,,..,.,,:::iri:. .,:irsr:,.;i.
 *        ;;..,::::;;;;ri,,,.                    ..,,:;s1s1ssrr;,.;r,
 *        :;. ,::;ii;:,     . ...................     .;iirri;;;,,;i,
 *        ,i. .;ri:.   ... ............................  .,,:;:,,,;i:
 *        :s,.;r:... ....................................... .::;::s;
 *        ,1r::. .............,,,.,,:,,........................,;iir;
 *        ,s;...........     ..::.,;:,,.          ...............,;1s
 *       :i,..,.              .,:,,::,.          .......... .......;1,
 *      ir,....:rrssr;:,       ,,.,::.     .r5S9989398G95hr;. ....,.:s,
 *     ;r,..,s9855513XHAG3i   .,,,,,,,.  ,S931,.,,.;s;s&BHHA8s.,..,..:r:
 *    :r;..rGGh,  :SAG;;G@BS:.,,,,,,,,,.r83:      hHH1sXMBHHHM3..,,,,.ir.
 *   ,si,.1GS,   sBMAAX&MBMB5,,,,,,:,,.:&8       3@HXHBMBHBBH#X,.,,,,,,rr
 *   ;1:,,SH:   .A@&&B#&8H#BS,,,,,,,,,.,5XS,     3@MHABM&59M#As..,,,,:,is,
 *  .rr,,,;9&1   hBHHBB&8AMGr,,,,,,,,,,,:h&&9s;   r9&BMHBHMB9:  . .,,,,;ri.
 *  :1:....:5&XSi;r8BMBHHA9r:,......,,,,:ii19GG88899XHHH&GSr.      ...,:rs.
 *  ;s.     .:sS8G8GG889hi.        ....,,:;:,.:irssrriii:,.        ...,,i1,
 *  ;1,         ..,....,,isssi;,        .,,.                      ....,.i1,
 *  ;h:               i9HHBMBBHAX9:         .                     ...,,,rs,
 *  ,1i..            :A#MBBBBMHB##s                             ....,,,;si.
 *  .r1,..        ,..;3BMBBBHBB#Bh.     ..                    ....,,,,,i1;
 *   :h;..       .,..;,1XBMMMMBXs,.,, .. :: ,.               ....,,,,,,ss.
 *    ih: ..    .;;;, ;;:s58A3i,..    ,. ,.:,,.             ...,,,,,:,s1,
 *    .s1,....   .,;sh,  ,iSAXs;.    ,.  ,,.i85            ...,,,,,,:i1;
 *     .rh: ...     rXG9XBBM#M#MHAX3hss13&&HHXr         .....,,,,,,,ih;
 *      .s5: .....    i598X&&A&AAAAAA&XG851r:       ........,,,,:,,sh;
 *      . ihr, ...  .         ..                    ........,,,,,;11:.
 *         ,s1i. ...  ..,,,..,,,.,,.,,.,..       ........,,.,,.;s5i.
 *          .:s1r,......................       ..............;shs,
 *          . .:shr:.  ....                 ..............,ishs.
 *              .,issr;,... ...........................,is1s;.
 *                 .,is1si;:,....................,:;ir1sr;,
 *                    ..:isssssrrii;::::::;;iirsssssr;:..
 *                         .,::iiirsssssssssrri;;:.
 *		Code is far away from bug with the animal protecting
 *						���ޱ��ӣ���������BUG
 */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DR16_RECIVE.h"
#include "my_positionPID_bate.h"
#include "FPS_Calculate.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t time_every100us=0;
int time_every1s=0;
uint32_t time_every100ms=0;

//uint32_t task2_times=0;
//uint32_t task3_times=0;
//uint32_t pendsv_times=0;
//uint32_t sysclk_to_pendsv=0;
//int guance1;
//int guance2;
uint32_t zdcsjc=0;//can���մ���
uint32_t time7_times=0;//��ʱ��7�жϽ������
uint32_t task_init_times=0;//��ʼ���������д���
uint32_t task_can_times=0;//CAN�����������д���
uint32_t task_can2_times=0;//CAN2�����������д���

uint32_t task_debug_times=0;//��λ�������������д���
uint32_t task_controul_times=0;//�ܿ����������д���
int every_1s_times=0;//�������

float yaw_trage_angle=0;
float yaw_trage_angle2;

float yaw_trage_speed=0;
float PITCH_trage_angle=0;
float PITCH_trage_angle_2=0;

int PITCH_trage_speed=0;
int CHASSIS_trage_speed=0;
int CHASSIS_trage_speed_last=0;
int CHASSIS_trage_angle=0;

float PITCH_MAX_angle=0;
float PITCH_MIN_angle=0;

int CHASSIS_MAX_SPEED=9000;

int send_to_yaw=0;//���͸�yaw�������
int send_to_pitch=0;//���͸�pitch�������
int send_to_chassis=0;//���͸����̵�����
int send_to_chassis_special=0;//���͸����̵�����

bool use_special_send=0;
int send_to_SHOOT_R=0;//���͸��ҷ���Ħ���ֵ�����  +
int send_to_SHOOT_L=0;//���͸�����Ħ���ֵ�����  -
int send_to_driver=0;//���͸����̵��������

float PITCH_IMU_Kp=1;

int M_3508_error=0;

int SHOOT_L=0;//��Ħ���ֵ�Ŀ���ٶ�   Ӧ��Ϊ��ֵ
int SHOOT_R=0;//��Ħ���ֵ�Ŀ���ٶ�

bool HWswitch_R=1;
bool HWswitch_L=1;

bool HWswitch_R_last=1;
bool HWswitch_L_last=1;

int CHASSIS_L_MAX=510000;
int CHASSIS_R_MIN=0;

int ENCODER_L_MAX=0;//������ͨ�������µ����ֵ
int ENCODER_R_MIN=1000;	//������ͨ�������µ���Сֵ
int ENCODER_M_MID=0;//������ͨ�������µ��м�ֵ
int ENCODER_LONG=250000;//������ͨ�������µĹ������
int ENCODER_SPEED=250000;//��ε�ֵ
int M3508_3ms_ago;//3������ǰ��ֵ
int M3508_3ms_change;//3����ı��ֵ
int M3508_3ms_ago_total_angle;//3������ǰ��ֵ
int M3508_3ms_ago_speed;//3����ı��ֵ
float M3508_speed_angle_kp;//�Ƕ����ٶȵĹ�ϵ
float encoder_fbl_k=1.0;//�ֱ��ʵı�����ϵ
int ENCODER_ADD=0;//��������ε�ֵ���ϴε�ֵ�õ����ٶ�
int ENCODER_CHANGE=0;//BMQ�ı�ֵ

int ENCODER_ARRIVE_MAX=0;//�������ִ�����ֵ
int ENCODER_ARRIVE_MIN=0;//�������ִ����Сֵ

int32_t CHASSIS_L_MAX_by_ENCODER;//�����õ���߽�ֵ
int32_t CHASSIS_R_MIN_by_ENCODER;//�����õ��ұ߽�ֵ
int reverse_by_ENCODER=8000; //������ǰֵ
int speed_change=0;
int CHASSIS_MID=250000;

int driver_targe_speed=0;

short int speed_change_times=0;

float DEBUFF=0;

int uart_8_times=0;
int uart_3_times=0;


int time_3_times=0;

float Vision_RawData_Yaw_Angle=0;
float Vision_RawData_Pitch_Angle=0;

bool send_to_C;//DR16_ң����_�Ƿ��͸�C��
/*
������״̬���ݣ�10Hz ���ڷ���                              27(15)
ʵʱ�����������ݣ�50Hz ���ڷ���                            16(14)
�˺�״̬���ݣ��˺���������                               1
ʵʱ������ݣ��ӵ��������                               6
�ӵ�ʣ�෢���������л������Լ��ڱ������˷��ͣ�1Hz ���ڷ���   2
*/
bool send_to_C_JS_SHOOT;//����ϵͳ_��������_�Ƿ��͸�C��  
bool send_to_C_JS_HURT;//����ϵͳ_�˺�����_�Ƿ��͸�C��
bool send_to_C_JS_STATUS;//����ϵͳ_״̬����_�Ƿ��͸�C��
bool send_to_C_JS_HEAT=0;//����ϵͳ_״̬����_�Ƿ��͸�C��

bool send_to_C_IN_END=0;//�Ƿ� ���ڹ����ͷ_״̬����_�Ƿ��͸�C�� 100ms��һ��
bool send_to_C_IN_MID=0;// ���ڹ���м�_״̬����_�Ƿ��͸�C��  1��һ��

bool in_MID=0;//���ڹ���м��
bool in_END=0;//���ڹ����ͷ

bool last_in_MID=0;//�ոմ��ڹ���м��
bool last_in_END=0;//�ոմ��ڹ����ͷ

int send_to_C_times;
int send_to_C_STATUS_times=0;//��Ϊ״̬���ݷ�4�η���,���Լ�������һ��

int CH_TOTAL=0;
float Chassis_PowerLimit=1;
bool stop_chassic_output=0;
bool stop_CH_OP_BC_END=0;
int stop_CH_OP_BC_END_times=0;
bool stop_CH_OP_BC_LESS=0;
int stop_CH_OP_BC_LESS_times=0;
int CHASSIS_trage_speed_temp;
int JS_SEND_times=0;//����ϵͳ���ʹ���
int place_SEND_times=0;//����λ�÷��ʹ���

int M3508_acceleration_cycle=10;//���ٶȼ�������(ÿ�μ���ļ��)(ms)
int M3508_last_speed;//��һʱ�̵��ٶ�,���ڼ��ٶȼ���
float M3508_acceleration; //���ٶ�

int hurt_times_ago=9999;//���ǰ������,��ʼ�����ᴥ��
float KB_add_speed=1.4;//��ģʽ����
float KB_low_JB=30;//��ģʽ���幦������
float KB_high_JB=80;//��ģʽ���幦�ʻظ�Ŀ��

int usart_7_times=0;

bool disable_for_test=0;///Ϊ�˵���///Ϊ�˵���///Ϊ�˵���///Ϊ�˵���///Ϊ�˵���///Ϊ�˵���
bool state_Infrared_R_is_ok=0;//�ұߺ��⴫������ת����
bool state_Infrared_L_is_ok=0;//��ߺ��⴫������ת����

int ch4_DW_total=0;
int restart_times=0;//����ʱ��

int CAN2_SEND_TASK_times=0;//��������ʱ��
//driver  plate
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  MX_RNG_Init();
  MX_TIM5_Init();
  MX_UART8_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
   usart1_dr16_init();

  
//USART6->DR = '2'; 
    /*ʹ�ܶ�ʱ��1�ж�*/
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim3);
	
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
//	  	   NM_swj();
//	  osdelay();
	  HAL_Delay(1);
//	  NM_swj2();
	  	  HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//  uint32_t wait;
//uint32_t tickstart;
int my_ms_delay1(uint32_t ms_Delay)
	//ms_Delay=100
{
//	   tickstart = MY_GetTick();
//   wait = time_every100us+10*ms_Delay;

  /* Add a freq to guarantee minimum wait */
//  if (wait < HAL_MAX_DELAY)
//  {
//    wait += (uint32_t)(uwTickFreq);
//  }

  while(1)
  {
//	 if( time_every100us> wait)
//	 {
//		 	  guance1=time_every100us- wait;
//		 return 0;
//	 }
//	 	  guance2=guance1;
  }
	
}

void my_ms_delay2(uint32_t ms_Delay2)
	//ms_Delay=100
{
//	  uint32_t tickstart2 = time_every100us;
//  uint32_t wait2 = ms_Delay2*10;

  /* Add a freq to guarantee minimum wait */
//  if (wait < HAL_MAX_DELAY)
//  {
//    wait += (uint32_t)(uwTickFreq);
//  }

//  while((MY_GetTick() - tickstart2) < wait2)
//  {
//  }
	
}

uint32_t MY_GetTick(void)
{
//  return time_every100us;
	  return 1000;

}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
