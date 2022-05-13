#include "MY_SHOOT_CONTROL.h"
#include "main.h"
#include "my_IncrementPID_bate.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "Vision.h"
#include "calibrate_task.h"


#define SHOOT_SPEED_HIGH 6700//���� ��
int shoot_speed_text=6750;
int SHOOT=0;
int SHOOT_from_V=0;
bool vision_soot_last_is_same=0;
bool SHOOT_from_v_last=1;


int DW_FREE=0;
int DW_DOWN=0;
int DW_UP=0;
long M2006_targe_angle=0;
bool Driver_arrive=1;
int if_Driver_arrive_angle=5000;

bool weather_error_less_than_1=0;
	
allow_auto_shoot auto_shoot_condition;
int auto_shoot_condition_show;//�Զ��������չ��

void shoot_control(void)
{
	if(in_END==0&&in_MID==1)
	{
		auto_shoot_condition.not_in_track_end=1;//���ڹ��ĩ��
	}
	else
	{
		auto_shoot_condition.not_in_track_end=0;//���ڹ��ĩ��		
	}
	
	if(vision_shoot_times>1)
	{
		auto_shoot_condition.vision_shoot_is_continuous=1;//�Ӿ�����ָ����������
	}
	else
	{
		auto_shoot_condition.vision_shoot_is_continuous=0;//�Ӿ�����ָ����������		
	}
	
	if(VISION_Yaw_IMU_Angle_pid.Error>-1.0f||VISION_Yaw_IMU_Angle_pid.Error<1.0f)
	{
	    auto_shoot_condition.weather_angle_error_less_than_1=1;	//�Ƕ����С��һ
	}
	else
	{
	    auto_shoot_condition.weather_angle_error_less_than_1=0;	//�Ƕ����С��һ		
	}
	
	if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<200)//����û����200
	{
		auto_shoot_condition.heat_allow=1;//��������
	}
	else
	{
		auto_shoot_condition.heat_allow=0;//��������		
	}
	
	
	if(auto_shoot_condition.heat_allow==1/*��������*/
	 &&auto_shoot_condition.weather_angle_error_less_than_1==1	/*�Ƕ����С��һ*/ 
	 &&auto_shoot_condition.vision_shoot_is_continuous==1/*�Ӿ�����ָ����������*/
	 &&auto_shoot_condition.not_in_track_end==1/*���ڹ��ĩ��*/
	  )
	{
	auto_shoot_condition.ALL_condition_satisfaction=1;	//��������ȫ������
	}
	else
	{
	auto_shoot_condition.ALL_condition_satisfaction=0;			
	}
	auto_shoot_condition_show=0;
	if(auto_shoot_condition.heat_allow==1)
		auto_shoot_condition_show+=1;
	if(auto_shoot_condition.weather_angle_error_less_than_1==1)
		auto_shoot_condition_show+=10;
	if(auto_shoot_condition.vision_shoot_is_continuous==1)
		auto_shoot_condition_show+=100;	
	if(auto_shoot_condition.not_in_track_end==1)
		auto_shoot_condition_show+=1000;
	if(auto_shoot_condition.ALL_condition_satisfaction==1)
		auto_shoot_condition_show+=10000;
	
			if(DR16.rc.s_left==2)//ң�������� ������׼����
			{	
				ch4_DW_total+=DR16.rc.ch4_DW;

						if(DR16.rc.ch4_DW<=-100)//����
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET);

			}
			if(DR16.rc.ch4_DW>=200)//����
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET);
//					cali_sensor[0].cali_cmd = 1;//������У׼
			}
			 
			}
			if(VisionData.RawData.Beat==0)//
				{
		SHOOT_STOP_time++;
				}
			else if(VisionData.RawData.Beat==1)//
				{
		SHOOT_STOP_time=0;
				}
	
	if(DR16.rc.s_left==3)
	{
//		
					if(DR16.rc.ch4_DW==0)//����
					{
						DW_FREE++;
						DW_UP=0;DW_DOWN=0;
							M2006_targe_angle=M3508s[1].totalAngle;//�����������

					}	
if(DR16.rc.ch4_DW>=200)//����
			{	
			DW_DOWN++;	
		if(DW_DOWN==20)
			M2006_targe_angle+=(Driver_add/4);//8*3=24
				if(DW_DOWN%100==0&&DW_DOWN>200)
			M2006_targe_angle+=Driver_add;//8*3=24		
				
			}

			
if(DR16.rc.ch4_DW<=-100)//����
			{
								DW_UP++;//û�õ���

		if(DW_UP==100)
			M2006_targe_angle+=(Driver_add/10);//8*3=24


				
			}
			
			
		
		
	}
	
		if(DR16.rc.s_left==1)//ң��������  ���л�����
	{
		if(M3508s[1].totalAngle>(M2006_targe_angle*0.8-if_Driver_arrive_angle))
		Driver_arrive=1;

//									if(DR16.rc.ch4_DW<=-100)//����
//			{		
	if(DR16.rc.s_right==3||DR16.rc.s_right==2)
	{
M2006_targe_angle=M3508s[1].totalAngle;//�����������
shoot_times_for_limit=0;
	
	}
		if(DR16.rc.s_right==2)
		{
			targe_shoot_number=JS_RC_TIMES+every_shoot_number;
		}

		if(DR16.rc.s_right==1)
		{
			shoot_times_for_limit++;
#if 0
			if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<200)//����û����200
			{
			if (shoot_times_for_limit<50)
			{
					if(VisionData.RawData.Beat==1)
					{
						SHOOT_from_V++;
//						M2006_targe_angle+=Driver_add;//8*3=24  ��һ��
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					
				
			}
			else if(shoot_times_for_limit>=50)
			{
			shoot_times_for_limit=0;
			this_period_has_shoot=0;
				whether_shoot_in__this_period=0;
				M2006_targe_angle=M3508s[1].totalAngle;//����������� ������

			}
		}
			else if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat>=200)//��������200
		{
						if (shoot_times_for_limit<100)
			{
					if(VisionData.RawData.Beat==1)
					{
						SHOOT_from_V++;
//						M2006_targe_angle+=Driver_add;//8*3=24  ��һ��
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					
				
			}
			else if(shoot_times_for_limit>=110)
			{
			shoot_times_for_limit=0;
			this_period_has_shoot=0;
				whether_shoot_in__this_period=0;
				M2006_targe_angle=M3508s[1].totalAngle;////����������� ������

			}
		}
		if(whether_shoot_in__this_period==1&&this_period_has_shoot==0)//�ж�һ��:��������Ƿ���(�����) ���� ��������Ƿ��Ѿ�����(���û��)
				{
					if(JS_RC_TIMES<targe_shoot_number)//һ�δ���� �����ӵ�����������
					{
						M2006_targe_angle+=Driver_add;//8*3=24  ��һ��
						this_period_has_shoot_number++;
						this_period_has_shoot=1;
					}
				}
					if(VisionData.RawData.Beat==1)
					{
						SHOOT_from_V++;
					}
		
							if(Armour_lose_time>40)//40msû��⵽װ�װ�
				{				
					M2006_targe_angle=M3508s[1].totalAngle;//�����������
				}
					if(SHOOT_STOP_time>200)//200msû��⵽����ָ��
				{				
					M2006_targe_angle=M3508s[1].totalAngle;//�����������
				}

#endif
#if 0
						if (shoot_times_for_limit<200)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>2)//�����־λΪ1���������յ�4֡
					{
						SHOOT_from_V++;

						M2006_targe_angle+=Driver_add;//8*3=24  ��һ��
						
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>100)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//�����յ�10��ͣ��ָ�� �������Ŀ��Ƕ��ۼ�
					}
	
			}
			else if(shoot_times_for_limit>=200)
			{
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//����������� ��������ۼ�Ŀ��ֵһ�� ��ֹ����
			}
#endif
#if 0//����ĩ��
						if (shoot_times_for_limit<200)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>2)//�����־λΪ1���������յ�4֡
					{
						SHOOT_from_V++;
						if(	in_MID==1)//����ĩ��
						{
						M2006_targe_angle+=Driver_add;//8*3=24  ��һ��
						whether_shoot_in__this_period=1;
						}
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>100)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//�����յ�10��ͣ��ָ�� �������Ŀ��Ƕ��ۼ�
					}
	
			}
			else if(shoot_times_for_limit>=200)
			{
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//����������� ��������ۼ�Ŀ��ֵһ�� ��ֹ����
			}
#endif
#if 1//����ĩ��+���С��1��
						if (shoot_times_for_limit<200)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>2)//�����־λΪ1���������յ�4֡
					{
						SHOOT_from_V++;
						if(VISION_Yaw_IMU_Angle_pid.Error>1.0||VISION_Yaw_IMU_Angle_pid.Error<-1.0)
						{
						weather_error_less_than_1=0;	
						}
						else//������ֵС��1
						{
						weather_error_less_than_1=1;	
						}
						if(in_MID==1&&weather_error_less_than_1==1)//����ĩ��
						{
						M2006_targe_angle+=Driver_add/4;//8*3=24  ��һ��
						whether_shoot_in__this_period=1;
						}
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>100)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//�����յ�10��ͣ��ָ�� �������Ŀ��Ƕ��ۼ�
					}
	
			}
			else if(shoot_times_for_limit>=200)
			{
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//����������� ��������ۼ�Ŀ��ֵһ�� ��ֹ����
			}
#endif
#if 0
						if (shoot_times_for_limit<500)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>2)//�����־λΪ1���������յ�4֡
					{
						SHOOT_from_V++;
						if(Driver_arrive==1)
						{
						M2006_targe_angle+=Driver_add*0.8;//8*3=24  ��һ��
								Driver_arrive=0;
						}
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>10)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//�����յ�10��ͣ��ָ�� �������Ŀ��Ƕ��ۼ�
					}
	
			}
			else if(shoot_times_for_limit>=500)
			{
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//����������� ��������ۼ�Ŀ��ֵһ�� ��ֹ����
			}
#endif
					SHOOT_from_v_last=VisionData.RawData.Beat;


				
//					if(VisionData.RawData.Beat==0)
//								M2006_targe_angle=M3508s[1].totalAngle;//�����������

//					if(SHOOT_from_v_last==VisionData.RawData.Beat)
//						vision_soot_last_is_same=1;
//					else
//					{
//						vision_soot_last_is_same=0;
//					SHOOT_from_V=0;
//					}
					
					
//					if(vision_shoot_times>0)//����1���յ��Ӿ��ķ���ָ��
//					{
//					SHOOT_from_V=0;
//						vision_shoot_times=0;
//					}
				}
		
				DW_FREE=0;
					SHOOT++;				



				if(DW_FREE>20)
				{
					SHOOT++;
					DW_FREE=0;
				}
			}
		
//M2006_targe_speed=(DR16.rc.ch1*1.0/660.0)*(-1)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
//		if(	driver_targe_speed<0)
//		driver_targe_speed=-driver_targe_speed;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
			
M2006_targe_speed=P_PID_bate(&Driver_ANGLE_pid,M2006_targe_angle,M3508s[1].totalAngle);//M2006_targe_speedӦ�ô���0
	
	
send_to_2006=I_PID_Regulation(&Driver_I_PID,M2006_targe_speed,M3508s[1].realSpeed);
			
			
			

	if(DR16.rc.s_left==2||DR16.rc.s_left==0)	//ʧ�ܱ���
					{			
						M2006_targe_angle=M3508s[1].totalAngle;//�����������
						Driver_arrive=1;//ʧ��,ͬ��Ҳ�ֵ�Ŀ��
					}
	//û��ң������ң�����������µ�λʱ����ʧ��
	//�����к����ϵ�λʱ
	
	
	
	
//		if(DR16.rc.s_left==1)//ң��������  ����
//SHOOT_L_speed=500;
//	if(DR16.rc.s_left==1)//ң��������  ����
//					{
//SHOOT_L_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//ң�������ٶ�Ŀ��ֵ ��ѡһ	
//if(DR16.rc.ch3<-600)	
			if(DR16.rc.s_left==3)//ң��������  ����
     	{		
			
			if (DR16.rc.s_right == 3) //�м䵵λ 
			SHOOT_L_speed=shoot_speed_text;
			else if(DR16.rc.s_right ==1)
			SHOOT_L_speed=500;//����
			else if(DR16.rc.s_right ==2)
			SHOOT_L_speed=0;
			
		}
		
					if(DR16.rc.s_left==1)//ң��������  ����
     	{				
			SHOOT_L_speed=-6800;//����Ħ�����ٶ�
		}
		
		
		
		
		SHOOT_R_speed=SHOOT_L_speed;	
		
		if(	SHOOT_R_speed<0)
			SHOOT_R_speed=-SHOOT_R_speed;//��Ħ�����ٶ�Ŀ��ֵӦ���� ֵ
		if(	SHOOT_L_speed>0)
			SHOOT_L_speed=-SHOOT_L_speed;//��Ħ�����ٶ�Ŀ��ֵӦ���� ֵ

send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_R_speed,M3508s[3].realSpeed);//gai//����

send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_L_speed,M3508s[2].realSpeed);   //����
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	//send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);

//	if(DR16.rc.s_left==3)//ң��������  ����
//					{
//SHOOT_L=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
//				SHOOT_R=SHOOT_L;					
//		if(	SHOOT_L>0)
//		SHOOT_L=-SHOOT_L;//��Ħ�����ٶ�Ŀ��ֵӦ���Ǹ�ֵ
//		if(	SHOOT_R<0)
//		SHOOT_R=-SHOOT_R;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
//send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_L,M3508s[0].realSpeed);

//send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);
	
//					}
	
}

void driver_plate_control(void)
{
//	if(DR16.rc.s_left==3||DR16.rc.s_left==1)//ң��������  ���л�����

//driver_targe_speed=(DR16.rc.ch1*1.0/660.0)*(-1)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
//		if(	driver_targe_speed<0)
//		driver_targe_speed=-driver_targe_speed;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
//send_to_driver=I_PID_Regulation(&Driver_I_PID,driver_targe_speed,M3508s[2].realSpeed);





}







