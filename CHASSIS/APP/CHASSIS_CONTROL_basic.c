#include "CHASSIS_CONTROL_basic.h"
#include "math.h"
#include "rng.h"
#include "RM_JudgeSystem.h"
#include "CHASSIS_CONTROL_2.h"


#define HW_SWITCH_JR 2000//���ļ�����  2000  500
#define GD_LONG 999999
#define track_long 84500//31000Ϊ�̹��  84500Ϊ�����   -4548 80741
#define inspect_times 200 
int init_times=0;
int Encoder_last=0;

int send_to_chassis_JUST_MOVE=2000;//���͸����̵�����_�պ��㹻������

bool CHASSIS_L_MAX_new=0;//���ұ߽�ֵ�Ƿ����
bool CHASSIS_R_MIN_new=0;
CHASSIS_KEY key_message;
void switch_change(void)
{
			HWswitch_L			 = HAL_GPIO_ReadPin(GPIOA,HWswitch_1_Pin);
			HWswitch_R   		 = HAL_GPIO_ReadPin(GPIOA,HWswitch_2_Pin);
			//���仯���  ����
	if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //������紫����������ʱ�ĳ�ʼ���߼�
{
   	if(HWswitch_L!=HWswitch_L_last)
	{
			if(HWswitch_L_last==1)//		0<--1
			{
				CHASSIS_L_MAX=M3508s[3].totalAngle+HW_SWITCH_JR;	
CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine+616;		
				CHASSIS_L_MAX_new=1;//�߽�ֵ�Ѹ���
			}
	}
	if(HWswitch_R!=HWswitch_R_last)
	{
			if(HWswitch_R_last==1)//		1-->0
			{
				CHASSIS_R_MIN=M3508s[3].totalAngle-HW_SWITCH_JR;
				CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine-1590;
				CHASSIS_R_MIN_new=1;//�߽�ֵ�Ѹ���
			}
		
	}
	ENCODER_LONG=	CHASSIS_L_MAX-CHASSIS_R_MIN;

}
else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //������紫������ߺ��ұ߻�ʱ�ĳ�ʼ���߼�
{
	//�߼�������Ǻõ�,�Ǿ�ֻ�����ߵ���߽� ,������ʾ����  
	//������߽�ʱ,�õ�ǰֵ��ȥ�������
    //������:�������        -31970��37   (�ñ���������)  
	//�Ҵ�����������Ϊ-30419��-32009  1590
	//�󴫸���������λ-579��37        616    
	//�õ��ұ߽�
   	if(HWswitch_L!=HWswitch_L_last)
	{
			if(HWswitch_L_last==1)//		0<--1
			{
				
				CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine       -reverse_by_ENCODER;
				CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine-track_long-reverse_by_ENCODER;//200000���ֵ��Ҫ��,�ĳɹ������ֵ
				CHASSIS_L_MAX_new=1;//�߽�ֵ�Ѹ���
			}
		
	}
}
else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //������紫������߻��ұߺ�ʱ�ĳ�ʼ���߼�
{
	//�߼����ұ��Ǻõ�,�Ǿ�ֻ����ұߵ��ұ߽� ,������ʾ����  
	//�����ұ߽�ʱ,�õ�ǰֵ��ȥ�������
    //������:�������        -31856��78     (�ñ���������)
	//�õ���߽�
	if(HWswitch_R!=HWswitch_R_last)
	{
			if(HWswitch_R_last==1)////��һ����1,�Ǿ��Ǹոս��봫����������		1-->0
			{
				CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine+track_long-reverse_by_ENCODER;//200000���ֵ��Ҫ��,�ĳɹ������ֵ(���������)
				CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine      -reverse_by_ENCODER;

				CHASSIS_R_MIN_new=1;//�߽�ֵ�Ѹ���
			}
		
	}
}
else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==0)  //������紫������߻��ұߺ�ʱ�ĳ�ʼ���߼�
{
	//�߼����ұ��Ǻõ�,�Ǿ�ֻ����ұߵ��ұ߽� ,������ʾ����  
	//�����ұ߽�ʱ,�õ�ǰֵ��ȥ�������
    //������:�������        -31856��78     (�ñ���������)
	//�õ���߽�    send_to_chassis_special
	init_times++;
					if(DR16.rc.s_left==1)//���ϵ�λ      //���û��⵽�õ�������PID,�ı����Ŀ��Ƕ�
					{
		if(init_times>333)//��һ����������
		{
	if(init_times%inspect_times==0)//200ms���һ��
	{
//		if(send_to_chassis_special<0)
		ENCODER_CHANGE=Chassis_Encoder.totalLine-Encoder_last;
		if(CHASSIS_R_MIN_new==0)
		{
			if(abs(ENCODER_CHANGE)<2000)
			{	CHASSIS_R_MIN_new=1;
//								CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine      -reverse_by_ENCODER;
								CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine;//��¼�µ�ǰֵ��Ϊ�ұ߽�/��Сֵ

			init_times=-666;
			}
		}
		else if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==0)
		{
						if(abs(ENCODER_CHANGE)<2000)
						{
				CHASSIS_L_MAX_new=1;
//				CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine       -reverse_by_ENCODER;
				CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine;//��¼�µ�ǰֵ��Ϊ��߽�/���ֵ

						init_times=0;
						}
		}
		
		Encoder_last=Chassis_Encoder.totalLine;
	}
		}
	
					}
					else
					{
		init_times=0;
				
					}
						
		use_special_send=1;
	

}
			HWswitch_L_last		=HWswitch_L;
			HWswitch_R_last		=HWswitch_R;
	DO_NOT_STOP.Point_of_3section.point_one=CHASSIS_R_MIN+(CHASSIS_L_MAX-CHASSIS_R_MIN)/3;
	DO_NOT_STOP.Point_of_3section.point_two=CHASSIS_L_MAX-(CHASSIS_L_MAX-CHASSIS_R_MIN)/3;
   if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_3section.point_one||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_3section.point_one)
   {
	DO_NOT_STOP.CHASSIS_AREA_NOW=FIRST_AREA;   
   }
   else if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_two||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_two)   
   {
		DO_NOT_STOP.CHASSIS_AREA_NOW=SECOND_AREA;   

	
	
   }
   
	DO_NOT_STOP.Point_of_4section.point_one=CHASSIS_R_MIN+(CHASSIS_L_MAX-CHASSIS_R_MIN)/4;
	DO_NOT_STOP.Point_of_4section.point_two=CHASSIS_R_MIN+(CHASSIS_L_MAX-CHASSIS_R_MIN)/2;
	DO_NOT_STOP.Point_of_4section.point_three=CHASSIS_L_MAX-(CHASSIS_L_MAX-CHASSIS_R_MIN)/4;
//   if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_one||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_one)
//   {
//	DO_NOT_STOP.CHASSIS_AREA_NOW=FIRST_PERCENT;   
//   }
//   else if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_two||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_two)   
//   {
//		DO_NOT_STOP.CHASSIS_AREA_NOW=SECOND_PERCENT;   
//   
//   }
//   else if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_three||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_three)   
//   {
//		DO_NOT_STOP.CHASSIS_AREA_NOW=THREE_PERCENT;   
//   }
//   else
//   {
//		DO_NOT_STOP.CHASSIS_AREA_NOW=FOUR_PERCENT;   	   
//   }
   if(DO_NOT_STOP.CHASSIS_AREA_LAST==DO_NOT_STOP.CHASSIS_AREA_NOW)
   {
	   DO_NOT_STOP.This_area_stay_times++;
   }
   else
   {
	   DO_NOT_STOP.This_area_stay_times=0;	   
   }
   DO_NOT_STOP.CHASSIS_AREA_LAST=DO_NOT_STOP.CHASSIS_AREA_NOW;
   
}

void star_and_new()
{
	if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //������紫����������ʱ�ĳ�ʼ���߼�
	{
		if(CHASSIS_R_MIN_new==0||CHASSIS_L_MAX_new==0	)	//ֻ�е��߽�ֵ�������˲Ż�  ������ʼѲ��	
		{
					if( HWswitch_L==0)// �����Ӧ���ˣ������˶�
					CHASSIS_trage_angle=-9990000;
					else if(HWswitch_R==0)//	�ҹ���Ӧ���ˣ������˶�
					CHASSIS_trage_angle=9990000;
					
				if(DR16.rc.s_left==1)//���û��⵽�õ�������PID,�ı����Ŀ��Ƕ�
				{			
				P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);
				CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*0.8;//˫��	//�������������Ǻõ�,�ٶȿ��Կ�һ��
				}
		}
	}	
	
	else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //������紫������ߺ��ұ߻�ʱ�ĳ�ʼ���߼�
	{
		if(CHASSIS_L_MAX_new==0)//��ߵĵĴ�������û��⵽
		{
		CHASSIS_trage_angle=9990000;//ֻ����ߵĴ������Ǻõ�,���������˶�
		
				if(DR16.rc.s_left==1)//���ϵ�λ      //���û��⵽�õ�������PID,�ı����Ŀ��Ƕ�
				{			
				P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);
				CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*0.5;//˫��	//ֻ��һ�����������Ǻõ�,�ٶȿ�����һ��
				}
		}
	}
	
	else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //������紫������߻��ұߺ�ʱ�ĳ�ʼ���߼�
	{
		if(CHASSIS_R_MIN_new==0)//�ұߵĵĴ�������û��⵽
		{
		CHASSIS_trage_angle=-9990000;//ֻ���ұߵĴ������Ǻõ�,���������˶�
		
				if(DR16.rc.s_left==1)//���ϵ�λ      //���û��⵽�õ�������PID,�ı����Ŀ��Ƕ�
				{			
				P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);
				CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*0.5;//˫��	//ֻ��һ�����������Ǻõ�,�ٶȿ�����һ��
				}
		}
		
	}
		else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==0)  //������紫������߻��ұߺ�ʱ�ĳ�ʼ���߼�
	{
		if(CHASSIS_R_MIN_new==0)//�ұߵĵĴ�������û��⵽
		{
		CHASSIS_trage_speed=-1500;	
//		send_to_chassis_special=-send_to_chassis_JUST_MOVE;//�Ѹպö��������ٶ����ұ߽�/��Сֵ�߽��˶�
			
//				if(DR16.rc.s_left==1)//���ϵ�λ      //���û��⵽�õ�������PID,�ı����Ŀ��Ƕ�
//				{	
//					
//				}
							CHASSIS_MOTOR_SPEED_pid.Max_result=1900;
			CHASSIS_MOTOR_SPEED_pid.Min_result=-1900;
		}
		else if(CHASSIS_L_MAX_new==0)//��ߵĵĴ�������û��⵽
		{
			
//		send_to_chassis_special=send_to_chassis_JUST_MOVE;//�Ѹպö��������ٶ�����߽�/���ֵ�߽��˶�
				CHASSIS_trage_speed=1500;	
				CHASSIS_MOTOR_SPEED_pid.Max_result=1900;
			CHASSIS_MOTOR_SPEED_pid.Min_result=-1900;
//				if(DR16.rc.s_left==1)//���ϵ�λ      //���û��⵽�õ�������PID,�ı����Ŀ��Ƕ�
//				{	
//					
//				}
		}
		else
		{
						CHASSIS_MOTOR_SPEED_pid.Max_result=14000;
			CHASSIS_MOTOR_SPEED_pid.Min_result=-14000;	
			
		}
		
		
		
		use_special_send=1;
	}
}



