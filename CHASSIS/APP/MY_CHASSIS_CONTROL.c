#include "MY_CHASSIS_CONTROL.h"
#include "CHASSIS_CONTROL_2.h"

#include "math.h"
#include "rng.h"
#include "RM_JudgeSystem.h"
//��3508����ķ��������,��PA8
//��3508��С�ķ������ұ�,��PA9
Ramp_Struct CHASSIS;

bool Random_CHASSIS_CHOOSE=1;//�Ƿ�ѡ�����ģʽ
bool Cruise_CHASSIS_CHOOSE=0;//�Ƿ�ѡ��Ѳ��ģʽ
int arrive_speed_times=0;
int disable_times=0;
int disable_targe_times=100;//ʧ�ܳ���ʱ��:100  150  200 250  300
int next_disable_start_times=200;//�´�ʧ�ܼ��ʱ��:200  300 400 500
int whether_change_direction=0;//����ʹ��ʱ�Ƿ����
CH_DO_NOT_STOP_AT_ONE_AREA DO_NOT_STOP;//��Ҫ��ͬһ����ͣ������
void CHASSIS_CONTROUL(void)
{
	#if PID_CHASSIS_MOTOR
			#if 1	//���������Ѳ���˶�
					if(DR16.rc.s_left==1)//�Զ�����
					{
						

	if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //������紫����������ʱ�ĳ�ʼ���߼�
	{					
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//ֻ�е��߽�ֵ�������˲Ż�  ������ʼѲ��	
				{
					
				if(DR16.rc.ch4_DW<=-400)//����
				{
				Random_CHASSIS_CHOOSE=1;//��ѡ�����ģʽ
				Cruise_CHASSIS_CHOOSE=0;
				}
				if(DR16.rc.ch4_DW>=400)//����
				{
				Cruise_CHASSIS_CHOOSE=1;//��ѡ��Ѳ��ģʽ
				Random_CHASSIS_CHOOSE=0;	
				}  //��ʽ��������Ҫ�л�Ѳ��ģʽ,��һֱ����ͺ���
				
				if(Cruise_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
//				Cruise_CHASSIS();//Ѳ��ģʽ
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//��ѡ�����ģʽ
				Random_CHASSIS();//���ģʽ
//				CHASSIS_trage_speed=0;//����//�������Ժ�ȡ��ע��	
				}
	}
	else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //������紫������ߺ��ұ߻�ʱ�ĳ�ʼ���߼�
	{
				if(CHASSIS_L_MAX_new==1	)	//ֻ�е���߽�ֵ�������˲Ż�  ������ʼѲ��	
				{
					
//				if(DR16.rc.ch4_DW<=-400)//����
//				{					
//				Random_CHASSIS_CHOOSE=1;//��ѡ�����ģʽ
//				Cruise_CHASSIS_CHOOSE=0;
//				}	
//				if(DR16.rc.ch4_DW>=400)//����
//				{
//				Cruise_CHASSIS_CHOOSE=1;//��ѡ��Ѳ��ģʽ
//				Random_CHASSIS_CHOOSE=0;		
//				} //��ʽ��������Ҫ�л�Ѳ��ģʽ,��һֱ����ͺ���
				
				if(Cruise_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
//				Cruise_CHASSIS();//Ѳ��ģʽ
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//��ѡ�����ģʽ
				Random_CHASSIS();//���ģʽ
//				CHASSIS_trage_speed=0;//����//�������Ժ�ȡ��ע��	
				}		
		
	}
	else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //������紫������߻��ұߺ�ʱ�ĳ�ʼ���߼�
	{
				if(CHASSIS_R_MIN_new==1	)	//ֻ�е���߽�ֵ�������˲Ż�  ������ʼѲ��	
				{
					
//				if(DR16.rc.ch4_DW<=-400)//����
//				{					
//				Random_CHASSIS_CHOOSE=1;//��ѡ�����ģʽ
//				Cruise_CHASSIS_CHOOSE=0;
//				}	
//				if(DR16.rc.ch4_DW>=400)//����
//				{
//				Cruise_CHASSIS_CHOOSE=1;//��ѡ��Ѳ��ģʽ
//				Random_CHASSIS_CHOOSE=0;		
//				} //��ʽ��������Ҫ�л�Ѳ��ģʽ,��һֱ����ͺ���
				
				if(Cruise_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
//				Cruise_CHASSIS();//Ѳ��ģʽ
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//��ѡ�����ģʽ
				Random_CHASSIS();//���ģʽ
//				CHASSIS_trage_speed=0;//����//�������Ժ�ȡ��ע��	
				}				
		
	}
		if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==0)  //������紫����������ʱ�ĳ�ʼ���߼�
	{					
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//ֻ�е��߽�ֵ�������˲Ż�  ������ʼѲ��	
				{
					
				if(DR16.rc.ch4_DW<=-400)//����
				{
				Random_CHASSIS_CHOOSE=1;//��ѡ�����ģʽ
				Cruise_CHASSIS_CHOOSE=0;
				}
				if(DR16.rc.ch4_DW>=400)//����
				{
				Cruise_CHASSIS_CHOOSE=1;//��ѡ��Ѳ��ģʽ
				Random_CHASSIS_CHOOSE=0;	
				}  //��ʽ��������Ҫ�л�Ѳ��ģʽ,��һֱ����ͺ���
				
				if(Cruise_CHASSIS_CHOOSE==1)//��ѡ��Ѳ��ģʽ
//				Cruise_CHASSIS();//Ѳ��ģʽ
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//��ѡ�����ģʽ
				Random_CHASSIS();//���ģʽ
//				CHASSIS_trage_speed=0;//����//�������Ժ�ȡ��ע��	
				}
	}
					}
			#endif
//	if(DR16.rc.s_left==3||DR16.rc.s_left==1)//ң��������  ���м�
	if(DR16.rc.s_left==3)//ң��������  ���м�
	{
	CHASSIS_trage_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*CHASSIS_MAX_SPEED;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
	}
if(0)//�����ٶȵ�б��
{	
CHASSIS.Current_Value=M3508s[3].realSpeed;					
CHASSIS.Target_Value=CHASSIS_trage_speed;
CHASSIS_trage_speed_temp=Ramp_Function(&CHASSIS);
					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
	
//	CHASSIS_trage_speed_temp=0;//ʼ�������ڹ����//�������Ժ�ȡ��ע��
					P_PID_bate(&CHASSIS_MOTOR_SPEED_pid, CHASSIS_trage_speed_temp,M3508s[3].realSpeed);
	send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result;
}
if(1)//�ٶ�û��б��
{

					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
					P_PID_bate(&CHASSIS_MOTOR_SPEED_pid, CHASSIS_trage_speed,M3508s[3].realSpeed);
	send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result;	
}
//					Power_Calculate();
//					send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result*Chassis_PowerLimit;
#endif
	

	
}




Random_t RANDOM_CHASSIS;

uint16_t Random_CHANGE_times = 400; //  PS:Random_CHANGE_times=600 ��ô600*3 (3ms��һ���������) 1.8�������һ���Ƿ����

const uint8_t Random_Proportion = 40;      //�������(100-Random_Proportion)  ����30 ��ô���������������30�ͱ���  ��ô�������Ϊ70%
const uint16_t Random_CHANGE_speed = 2000;      //�ٴα���Ҫ�ﵽ����ٶ�����

//���ģʽ
void Random_CHASSIS(void)
{
//    if (abs(CHASSIS_trage_speed) != 6000*Chassis_PowerLimit)
//    {
//        CHASSIS_trage_speed = 3000*Chassis_PowerLimit;//����˶��Ļ����ٶ�
//    }//����˶�   ��ʼ���ٶ�   ��Random_Velocity�������˶�
    RANDOM_CHASSIS.number = Get_RandomNumbers_Range(0, 100);
//					if(M3508s[3].totalAngle>(CHASSIS_R_MIN+100000)&&M3508s[3].totalAngle<(CHASSIS_L_MAX-100000))//��ʵ��ȷ����Զ���� ��ʮ����ʮ��

//    RANDOM_CHASSIS.sampling++;
	
	#if 1
	if(	abs(M3508s[3].realSpeed) >Random_CHANGE_speed)
	    RANDOM_CHASSIS.sampling++;
//	if(DO_NOT_STOP.This_area_stay_times>1000)//��ͬһ����ͣ������3s,��ʼ��ͼ  ʧ��ʱ���ֵ̫����,�л����Զ�_����˶�
//	{
//		RANDOM_CHASSIS.sampling=0;
//		arrive_speed_times=0;
//					stop_CH_OP_BC_LESS=0;

//	}
	
//		if(	arrive_targe_angle==1	)
//	    RANDOM_CHASSIS.sampling++;
	#endif
						Power_Calculate();
if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //������紫����������ʱ�Ĺ��ĩ�����߼�
{
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+9999999))
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
				RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
			}
			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-9999999))//����߽���� ��ʮ����ʮ��
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
				RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
			}//ֻҪ��������⵽�˾ͷ���
			
//						if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
//				//�ںϱ�����,reverse_by_ENCODER�Ǳ�����ǰֵ
//			{
//				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
//			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
//			}
//						if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))
//				//�ںϱ�����,reverse_by_ENCODER�Ǳ�����ǰֵ
//			{
//				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
//			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
//			}
			
}	
else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //������紫����   ��ߺ� �ұ߻�   ʱ�Ĺ��ĩ�����߼�
{
			if(HWswitch_L==0)//����߽���� ��ʮ����ʮ��
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
			}//ֻҪ��ߺõĴ�������⵽�˾ͷ���
			
			if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
				//�ںϱ�����,reverse_by_ENCODER�Ǳ�����ǰֵ
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
			}
	
}
	else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //������紫������߻��ұߺ�ʱ�Ĺ��ĩ�����߼�
	{
	
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+9999999))
				//ֻҪ�ұߺõĴ�������⵽�˾ͷ���
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
				RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
			}
			
			if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))
				//�ںϱ�����,reverse_by_ENCODER�Ǳ�����ǰֵ
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
			}
			
	}
	else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==0)  //������紫������߻��ұߺ�ʱ�Ĺ��ĩ�����߼�
	{
					if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
			{
			arrive_targe_angle=0;
			stop_CH_OP_BC_LESS=0;
				stop_CH_OP_BC_END_times=0;
								xunen_times++;
						CHASSIS_trage_speed_last=4000;

//							stop_chassic_output=1;	
//							  	  HAL_Delay(3000);
//							stop_chassic_output=0;
//				if(last_Speed>M3508s[3].realSpeed&&M3508s[3].realSpeed>0)//�Ѿ�������
								if(M3508s[3].realSpeed>0)//�Ѿ�������
{
	speed_has_change=1;
}

if(speed_has_change==0)
{
	
					if(Chassis_Encoder.totalLine>(CHASSIS_R_MIN_by_ENCODER+2000))
				{
					
								CHASSIS_trage_speed=-6000;
				stop_CH_OP_BC_END=0;

				}
				else
				{
					
						CHASSIS_trage_speed=-4000;
			stop_CH_OP_BC_END=1;

				}


}	
else
{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=9900000;
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;

}
	if(	xunen_times>3000)//˵����������,�϶��ǿ�����,������,ֱ����
	{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=9900000;
						CHASSIS_trage_speed=4000*Chassis_PowerLimit;

	}
			}
			
else if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))

//			else if(M3508s[3].totalAngle>(ENCODER_L_MAX-10000))//
//			else if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-3000))//
			{
				
								CHASSIS_trage_speed_last=-4000;
			arrive_targe_angle=0;
			stop_CH_OP_BC_LESS=0;
				stop_CH_OP_BC_END_times=0;
				xunen_times++;
//							stop_chassic_output=1;	
//							  	  HAL_Delay(3000);
//							stop_chassic_output=0;	
//				if(M3508s[3].realSpeed<0&&M3508s[3].realSpeed>last_Speed)//�ٶ�һ����Ҫ�ٶ��½�Ҳ���ǵ����쵽��ſ�ʼ��������ʱ��,Ȼ���������
				if(M3508s[3].realSpeed<0)//ֻҪ�ٶ�һ���Ϳ�ʼ��������ʱ��,Ȼ���������

{
	
	speed_has_change=1;
}
if(speed_has_change==0)
{
	
					if(Chassis_Encoder.totalLine<(CHASSIS_L_MAX_by_ENCODER-2000))
				{
					
	stop_CH_OP_BC_END=0;
					CHASSIS_trage_speed=6000*Chassis_PowerLimit;

				}
				else
				{
					
				CHASSIS_trage_speed=4000;
			stop_CH_OP_BC_END=1;

				}
//	if(	xunen_times>4)//���ɳ���ǰ���ٵ�ʱ��
//	{
//xunen_percent=0.7;		
//	}
//	else 
//	{
////xunen_percent=1.0;
//	}	
		
}
else
{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=-9900000;
					CHASSIS_trage_speed=-4000*Chassis_PowerLimit;

}
	if(	xunen_times>3000)//˵����������,�϶��ǿ�����,������,ֱ����
	{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=-9900000;
						CHASSIS_trage_speed=-4000*Chassis_PowerLimit;

	}
			}
						else//������������֮��
			{
//				xunen_percent=1.5;
//				if(M3508s[3].realSpeed==0)//��֪����ֹʱ�ٶ��ܷ��ȶ���0?
				if(abs(M3508s[3].realSpeed)<100)//�Ǿ���100������Ϊ��ֹ�жϵ�����
				stop_CH_OP_BC_END_times++;
				else
					stop_CH_OP_BC_END_times=0;
				
				if(stop_CH_OP_BC_END_times>2000)//��������û��ʱ��ͳ��˹���߽��ж�
					//�����ʧ�ܵ�/�ֶ����л�����ʱ�ٶ�Ŀ��ֵ:CHASSIS_trage_speed Ϊ0
				{
					stop_CH_OP_BC_END=0;//
					CHASSIS_trage_speed=CHASSIS_trage_speed_last;
				}
				speed_has_change=0;
				xunen_times=0;
				
				
				    if (RANDOM_CHASSIS.sampling >= Random_CHANGE_times)
    {
        if (RANDOM_CHASSIS.number >= Random_Proportion)//�Ƿ����
        {
            CHASSIS_trage_speed = -CHASSIS_trage_speed;
			arrive_targe_angle=0;
			stop_CH_OP_BC_LESS=0;
			speed_change_times++;
			if(CHASSIS_trage_speed>2000)//��ֵ
			{
			CHASSIS_trage_speed_last=4000;	
			}
			if(CHASSIS_trage_speed<-2000)//��ֵ
			{
			CHASSIS_trage_speed_last=-4000;	
			}
        }
        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
    }
				if(CHASSIS_trage_speed>0)
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
			}
			if(CHASSIS_trage_speed<0)
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
			}
	
						if(abs(CHASSIS_trage_speed)>2500)
					{
						if(abs(M3508s[3].realSpeed)>(abs(CHASSIS_trage_speed)-200))
						{
						arrive_speed_times++;		
						}
					}
					if(arrive_speed_times>250)//�й���ʧ��
					{
						if(abs(M3508s[3].realSpeed)<(abs(CHASSIS_trage_speed)-2000))
						{
						disable_times++;	
						}
						stop_CH_OP_BC_LESS=1;
//						CHASSIS_trage_speed=0;
					}	
					if(disable_times>50)//ÿ��ʧ��ʱ��:
						{
							arrive_speed_times=0;
							disable_times=0;
							stop_CH_OP_BC_LESS=0;
						}
				
				
				
				
			}
//			if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
//				//�ںϱ�����,reverse_by_ENCODER�Ǳ�����ǰֵ
//			{
//				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
//			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
//			}
//			
//			if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))
//				//�ںϱ�����,reverse_by_ENCODER�Ǳ�����ǰֵ
//			{
//				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
//			        RANDOM_CHASSIS.sampling = 0;//�����������500�βŻ����һ�α����ж�
//			}
			
		
		
		
		
		
		
		
		
	}



	
}


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max)
{
	uint32_t rng_number;

	rng_number = HAL_RNG_GetRandomNumber(&hrng);
	
	return rng_number % (max - min + 1) + min;
}


void Cruise_CHASSIS(void)//		cruise	Ѳ��
{
						
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
			CHASSIS_trage_angle=9900000;
			
			
			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//��ʵ��ȷ����Զ���� ��ʮ����ʮ��
			CHASSIS_trage_angle=-9900000;
			
			P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle
			CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//˫��
			
				CHASSIS_MID=(CHASSIS_R_MIN+CHASSIS_L_MAX)/2;
				//CHASSIS_MID-CHASSIS_R_MIN   һ���г�
				DEBUFF=abs(M3508s[3].totalAngle-CHASSIS_MID)/(CHASSIS_MID-CHASSIS_R_MIN);
				speed_change=DEBUFF*CHASSIS_trage_speed*0.7;		//�������ٷ�֮70
				CHASSIS_trage_speed=CHASSIS_trage_speed-speed_change;//�������м�죬������
	
	
	
	
//    if (fabs(Chassis.Velocity.temp_Speed) != Cruise_Velocity)
//    {
//        Chassis.Velocity.temp_Speed = Cruise_Velocity;
//    }
}
Encoder_t Chassis_Encoder;
//��ȡ������ֵ����
void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab)
{
	
	Chassis_Encoder->realValue_AB = (short)__HAL_TIM_GET_COUNTER(htim_ab);
	
	if(Chassis_Encoder->realValue_AB - Chassis_Encoder->lastValue_AB < -3600)
	{
		Chassis_Encoder->Counts ++;
	}
	if(Chassis_Encoder->lastValue_AB - Chassis_Encoder->realValue_AB < -3600)
	{
		Chassis_Encoder->Counts --;
	}
	
	Chassis_Encoder->totalLine = Chassis_Encoder->realValue_AB + Chassis_Encoder->Counts * OneLoop_LineNumber;
	
	Chassis_Encoder->lastValue_AB = Chassis_Encoder->realValue_AB;
	M3508_3ms_change=M3508s[3].totalAngle-M3508_3ms_ago;
	ENCODER_SPEED=Chassis_Encoder->totalLine-Chassis_Encoder->TargerLine;
//	ENCODER_ADD=Chassis_Encoder->realValue_AB-Chassis_Encoder->lastValue_AB;
	if(M3508_3ms_change!=0)//���˶�
	{
	encoder_fbl_k=	(ENCODER_SPEED*1.0)/M3508s[3].realSpeed;
	}
	
//	int M3508_3ms_ago_total_angle;//3������ǰ��ֵ
//int M3508_3ms_ago_speed;//3����ı��ֵ
//float M3508_speed_angle_kp;//�Ƕ����ٶȵĹ�ϵ
	
	M3508_speed_angle_kp=M3508s[3].realSpeed/1.0/(M3508s[3].totalAngle-M3508_3ms_ago);//2.45
	Chassis_Encoder->TargerLine=Chassis_Encoder->totalLine;
	M3508_3ms_ago=	M3508s[3].totalAngle;
    M3508_3ms_ago_speed=M3508s[3].realSpeed;
}



/***************************************
  * @brief  :���̹��ʼ���
  * @param  :Judge_DATA.Power,Judge_DATA.Power_Buffer,Power_MAX,Power_Buffer_MAX
****************************************/
void Power_Calculate()
{
	static uint8_t flag=0;
	static bool flag_kb=0;//��֤�����ģʽʱ,�����ڻָ�����

	if(ext_power_heat_data.data.chassis_power_buffer < 150)flag = 1;  
	//���幦��С�� 50  ��ʼ���ƹ���
	if(flag==1 && ext_power_heat_data.data.chassis_power_buffer > 180)flag=0;
	//���幦�����ӵ�  150  �������ƹ���
	
	if(flag==1)Chassis_PowerLimit = 1.00f;  //���ƹ��� �ڷ������ǰ ����0.65
	else Chassis_PowerLimit =1.3500f ;//; 0.8        //�������ƹ��� �ڷ������ǰ ����1.0
	
	if(hurt_times_ago<2000)//�����к�3s��
	{
		if(flag_kb==0)
		{
			flag=0;//��֤�����ģʽʱ,�����ڻָ�����
			flag_kb=1;
		}
//						Chassis_PowerLimit=KB_add_speed;//�񱩼��ٶ�

	if(ext_power_heat_data.data.chassis_power_buffer < KB_low_JB)flag = 1;  
	//���幦��С�� 50  ��ʼ���ƹ���
	if(flag==1 && ext_power_heat_data.data.chassis_power_buffer > KB_high_JB)flag=0;
	//���幦�����ӵ�  150  �������ƹ���
	
	if(flag==1)Chassis_PowerLimit = 1.15f;  //���ƹ��� �ڷ������ǰ ����0.65
	else Chassis_PowerLimit = 1.80;         //�������ƹ��� �ڷ������ǰ ����1.0
		
		
		Random_CHANGE_times=400;//�������Ƶ��
	}
	else
	{
		flag_kb=0;
				Random_CHANGE_times=700;//�������Ƶ�ʱ��

	}
	
}













