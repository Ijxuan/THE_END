#ifndef CHASSIS_CAN_H
#define CHASSIS_CAN_H

#include "DR16_RECIVE.h"

#include "M3508.h"
#include "my_positionPID_bate.h"
//#include "DJI_C_IMU.h"
#include "User_math.h"

void CHASSIS_CONTROUL(void);

void switch_change(void);

void Random_CHASSIS(void);

void Cruise_CHASSIS(void);
void Power_Calculate();


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max);
void Power_Calculate();

typedef struct 
{
    int point_one; //��1
    int point_two; //��2
}Trisection;//3�ȷֵ�
typedef struct 
{
    int point_one; //��1
    int point_two; //��2
	int point_three; //��3
}Quadrisection;//�ĵȷֵ�Trisection
//����˶��ṹ��
typedef struct 
{
    uint32_t number; //�����
    uint16_t sampling; //ɨ��ʱ��
}Random_t;
//���̵�����
typedef enum
{
	FIRST_AREA = 0,
	SECOND_AREA = 1,
	THREE_AREA=2,
	FOUR_AREA=3
} CHASSIS_AREA;
//����˶��ṹ��
typedef struct 
{
	Trisection Point_of_3section;
	Quadrisection Point_of_4section;
    CHASSIS_AREA CHASSIS_AREA_NOW; //��ʱ�˿̵�����
    CHASSIS_AREA CHASSIS_AREA_LAST; //��һʱ�̵�����
	int This_area_stay_times;//������ͣ��ʱ��
    uint16_t every_change_inteval_TIME; //���ʱ��
}CH_DO_NOT_STOP_AT_ONE_AREA;
extern CH_DO_NOT_STOP_AT_ONE_AREA DO_NOT_STOP;//��Ҫ��ͬһ����ͣ������

extern Random_t RANDOM_CHASSIS;

typedef struct
{
			
	//ֱ����������
	int16_t realValue_AB;
	
	int32_t TargerLine;
	
	int16_t lastValue_AB;
	
	int32_t Counts;
	int32_t totalLine;

	P_PID_t P_PID;
	
}Encoder_t;
extern Encoder_t Chassis_Encoder;
extern bool CHASSIS_L_MAX_new;//���ұ߽�ֵ�Ƿ����
extern bool CHASSIS_R_MIN_new;
extern Ramp_Struct CHASSIS;

void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab);

#define OneLoop_LineNumber (1024*4)//������һȦ

#define chassis_radom_mode_one 0


#define chassis_radom_mode_two 1//�ﵽĿ���ٶȾͼ���




#define chassis_radom_mode_three 0

#define chassis_radom_mode_four 0


#endif

