#include "MY_CLOUD_CONTROL.h"

cloud_control_mode cloud_mode;

void cloud_control(void)
{
//	
//								if(DR16.rc.s_left==1)//YAW����Ƶ�λ
//							{
//							yaw_trage_angle+=(DR16.rc.ch0/660.0)/-3;//YAW��ң��������
//							if(DR16.rc.ch4_DW<=-400)//����
//							yaw_trage_angle=(CLOUD_enable_imu+200.0);//�����ǽ��ٶ����Ϊ140?
//							if(DR16.rc.ch4_DW>=400)//����
//							yaw_trage_angle=(CLOUD_enable_imu-200.0);
//							
//							/*
//							����Ŀ���ԾҪ����140/0.03   4666
//							//��Ҫ����29000�������ò��ܵ�140�������ٶ�
//							//�ȸ�10000�Ľ�Ծ��
//							������800
//							*/
//							

//							}
///*					else
//					��-10000��30000���ʼ20000���������ٶΣ�Ȼ����10000��220�������10000�ļ���		
//					yaw_trage_angle+=(DR16.rc.ch3*1.0/660.0)*20;

//*/
//	
//
cloud_control_mode_choose();
 scan_cloud();
 if(DR16.rc.s_left!=3)
{
simulation_target_yaw=	DJIC_IMU.total_yaw;
}
 if(DR16.rc.s_left==3)
{
	if(DR16.rc.s_right==3)
	{
	simulation_target_yaw+= 0.018;   // 18��/1000����
	}
	else
	{
		simulation_target_yaw=	DJIC_IMU.total_yaw;
	}
	
}

							YAW_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	
//							PITCH_trage_angle=0;//����ˮƽλ��
							imu_angle();//ȥ���������ԣ�����  ����
//									PITCH_trage_angle=28;

							//��������Ư�޷�
						if(PITCH_trage_angle>PITCH_MAX_angle||PITCH_trage_angle==PITCH_MAX_angle)
													PITCH_trage_angle=PITCH_MAX_angle;
						if(PITCH_trage_angle<PITCH_MIN_angle||PITCH_trage_angle==PITCH_MIN_angle)
													PITCH_trage_angle=PITCH_MIN_angle;	
						
							PITCH_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}


void cloud_control_mode_choose(void)
{
	if(DR16.rc.s_left == 1)//����ʱ
	{
//		if(VisionData.RawData.Armour==1)

		if(Armour_lose_time>=3)//��ʧ3֡װ��
		{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);			
			
		}
		else
		{
			cloud_mode.control_mode_NOW=vision_mode;
			cloud_mode.control_mode_LAST=aoto_scan_mode;
			P_PID_Parameter_Clear(&Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&Yaw_IMU_Speed_pid);			
		}

		
	}
	else
	{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);			
	}
	
	
}


bool YAW_TARGE_ANGLE_ADD=1;
int arrive_targe_angle=0;
int TEXT_targe_SPEED=400;

void YAW_PID()
{
	
//									if(DR16.rc.s_left==1)//YAW����Ƶ�λ
//							{
//	if(DJIC_IMU.total_yaw>(CLOUD_enable_imu+360.0))	
//	{
//		
////TEXT_targe_SPEED		
////		arrive_targe_angle++;
////		if(arrive_targe_angle>20)
////		{
//cloud_text_add=0;
////			arrive_targe_angle=0;
////		}
//	}	
//	
//	if(DJIC_IMU.total_yaw<(CLOUD_enable_imu-360.0))	
//	{
//		
//		
////		arrive_targe_angle++;
////		if(arrive_targe_angle>20)
////		{
//cloud_text_add=1;
////			arrive_targe_angle=0;
////		}
//	}	
//	
//	if(cloud_text_add==1)//����
//	{
//yaw_trage_speed=TEXT_targe_SPEED;
//	}	
//	if(cloud_text_add==0)//�Ƕȼ�С,�ٶ�Ϊ��
//	{
//yaw_trage_speed=-TEXT_targe_SPEED;
//	}	
//	
////if(cloud_enable==0)
////{
////}
//							}	
		if (DR16.rc.s_left == 2 || DR16.rc.s_left == 0) //ʧ�ܱ���
		{
CLOUD_enable_moto=GM6020s[0].totalAngle;
CLOUD_enable_imu=DJIC_IMU.total_yaw;
		}
//						#if PID_MOTOR//YAW�����Ƕ�
//					P_PID_bate(&Yaw_Angle_pid, yaw_trage_angle,GM6020s[0].totalAngle);//GM6020s[EMID].totalAngle readAngle

//					yaw_trage_speed=Yaw_Angle_pid.result;//��Ծ


//							yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;//�������ٶ�
//					P_PID_bate(&Yaw_Speed_pid, yaw_trage_speed,GM6020s[0].readSpeed);//�õ�����ٶȱջ�
//	
//			                   send_to_yaw=Yaw_Speed_pid.result;

//					#endif
//							
					#if PID_YAW_IMU//YAW��������
									if(DR16.rc.s_left==3)//YAW����Ƶ�λ
							{
								if(cloud_mode.control_mode_NOW==aoto_scan_mode)//ɨ��PID
							yaw_trage_angle-=(DR16.rc.ch0/660.0)/10.0;//YAW��ң��������
//							CH0_TOTAL_in_con+=	DR16.rc.ch0;
//								if(DR16.rc.ch0!=0)
//								dr16_controul_times++;
							}
									if(DR16.rc.s_left==1)//YAW����Ƶ�λ
							{
//								if(VisionData.RawData.Armour==1)
//							yaw_trage_angle-=Vision_RawData_Yaw_Angle;//YAW��ң��������
								
				if(cloud_mode.control_mode_NOW==vision_mode)//ɨ��PID
				{
					yaw_trage_angle=DJIC_IMU.total_yaw-Vision_RawData_Yaw_Angle;//YAW��ң��������
				}			
//								else
//								yaw_trage_angle-=(DR16.rc.ch0/660.0)/10.0;//YAW��ң��������

							}							
//					Yaw_IMU_Angle_pid.Kp=-YAW_IMU_Kp;//���Թ��������ֵҪ���ϸ���
							
				if(cloud_mode.control_mode_NOW==aoto_scan_mode)//ɨ��PID
				{
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
		

					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
							
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
					
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
				}
				
				if(cloud_mode.control_mode_NOW==vision_mode)//����PID
				{
					P_PID_bate(&VISION_Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
		

					yaw_trage_speed=VISION_Yaw_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
							
					P_PID_bate(&VISION_Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
					
		                   send_to_yaw=VISION_Yaw_IMU_Speed_pid.result;
				}
				
				
//					send_to_pitch=(DR16.rc.ch3*1.0/660.0)*29000;
					#endif
//	
//	
	
}

void PITCH_PID()
{
//	#if PID_PITCH_MOTOR      //PITCH�����Ƕ�
//if(PITCH_trage_angle>7125)
//	PITCH_trage_angle=7125;
//if(PITCH_trage_angle<6435)
//	PITCH_trage_angle=6435;
//					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle,GM6020s[1].totalAngle);//GM6020s[EMID].totalAngle readAngle
////�����ǵ��ٶ�ֵ����С��
//					PITCH_trage_speed=PITCH_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
////					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
//				
//				P_PID_bate(&PITCH_Speed_pid, PITCH_trage_speed,GM6020s[1].readSpeed);
//				send_to_pitch=PITCH_Speed_pid.result;
//#endif	
//	
//	#if PID_PITCH_IMU		//PITCH��������
							if(DR16.rc.s_left==3)//PITCH����Ƶ�λ
							{
							PITCH_trage_angle+=(DR16.rc.ch1*1.0/660.0)*0.4;//ң�������ٶ�Ŀ��ֵ ��ѡһ

////							if(DR16.rc.ch4_DW<=-400)//����
////							PITCH_trage_angle=PITCH_MAX_angle-10;
////							if(DR16.rc.ch4_DW>=400)//����
////							PITCH_trage_angle=PITCH_MIN_angle+10;
//							
							}
														if(DR16.rc.s_left==1)//PITCH����Ƶ�λ
							{
								if(VisionData.RawData.Armour==1)
							PITCH_trage_angle=DJIC_IMU.total_pitch-Vision_RawData_Pitch_Angle;//YAW��ң��������
								else
								PITCH_trage_angle+=(DR16.rc.ch1/660.0)*0.4;//YAW��ң��������
							
							}

					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//�����ǵ��ٶ�ֵ����С��
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//��ȥ��ʵ��


//#endif
//	
}



void imu_angle()
{
//	PITCH_MAX_angle=DJIC_IMU.total_pitch+(7990-GM6020s[3].totalAngle)/8196.0*360.0;
//	PITCH_MIN_angle=DJIC_IMU.total_pitch+(7450-GM6020s[3].totalAngle)/8196.0*360.0;
	
	PITCH_MAX_angle=DJIC_IMU.total_pitch+(5080-GM6020s[3].totalAngle)/8191.0*360.0;
	PITCH_MIN_angle=DJIC_IMU.total_pitch+(3900-GM6020s[3].totalAngle)/8191.0*360.0;
			allow_angle=	PITCH_MAX_angle-PITCH_MIN_angle;

}

//2022-4-14:
//
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ15    6020ֵΪ4408      5170
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ-39.5      6020ֵΪ3154    3820

//2022-3-27:
//
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ23    6020ֵΪ8026
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ-31      6020ֵΪ6756
//2022-3-18:
//
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ-1.1    6020ֵΪ4008
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ-44      6020ֵΪ3028

//
//3070-3950
//3100-3930
//8017-7044
//8000-7400
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ6.5    6020ֵΪ8022
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ50      6020ֵΪ7020


//ɨ�躯��
//V1.0
//����������
//����ɨ��
//�����������е�λ��ʼɨ���˶�
//�����������£�ֹͣ�������ʱ���ۼ�ֵ
//t=0ʱ��PITCH���˶����м䣬Ȼ�������˶�
//��ɣ�����ֻ��YAW����˿��������PITCH��ʱĬ���г��м俪��ɨ��,PITCH��YAW���ٶȰ�
//V1.1
//PITCH��YAW���ٶȽ����,���Էֱ�����


		 bool scan_i_PITCH=1;
		 int scan_percent_PITCH=500;//0��1000,�ٷֱ�
		 int scan_time=0;

		 int scan_percent_YAW=500;//0��1000,�ٷֱ�
	 float YAW_START_ANGLE;//Sɨ�迪ʼʱYAW��Ƕ�

void scan_cloud(void)
{
//	if(DR16.rc.s_left==3)//���Ƶ�λ-ɨ��
			if(DR16.rc.s_left==1)//���Ƶ�λ-ɨ��
	{
//		if(DR16.rc.s_right==3)//���Ƶ�λ-ɨ�迪ʼ
		if(Armour_lose_time>1000)//�Ӿ�1��û����װ�װ�-ɨ�迪ʼ
		{
		 int scan_speed_PITCH=4;//PITCH��ɨ���ٶ�,��СΪ1
		int scan_speed_YWA=4;//YAW��ɨ���ٶ�,��СΪ1


		static bool scan_i_YAW=0;
			scan_time++;
if(scan_time%scan_speed_PITCH==0)//��ɨ���ٶȵ�������
{
	if(scan_percent_PITCH>0&&scan_percent_PITCH<1000)
	{
		if(scan_i_PITCH==0)//0����
		{
		scan_percent_PITCH++;
		}
		else//1��С
		{
		scan_percent_PITCH--;
		}
	}
	else if(scan_percent_PITCH<1)//�����±߽�,�л�������ģʽ
	{
		scan_i_PITCH=0;
		scan_percent_PITCH=1;
	}
	else//�����ϱ߽�,�л�����Сģʽ
	{
		scan_i_PITCH=1;
				scan_percent_PITCH=999;

	}
	
}
if(scan_time%scan_speed_YWA==0)//��ɨ���ٶȵ�������  scan_percent_YAW��0��1000֮�䲨��
{
	if(scan_percent_YAW>0&&scan_percent_YAW<1000)
	{
		if(scan_i_YAW==0)//0����
		{
		scan_percent_YAW++;
		}
		else//1��С
		{
		scan_percent_YAW--;
		}
	}
	else if(scan_percent_YAW<1)//�����±߽�,�л�������ģʽ
	{
		scan_i_YAW=0;
		scan_percent_YAW=1;
	}
	else//�����ϱ߽�,�л�����Сģʽ
	{
		scan_i_YAW=1;
				scan_percent_YAW=999;
	}
	
}	
			
PITCH_trage_angle=PITCH_MIN_angle+(allow_angle)*0.8*(scan_percent_PITCH/1000.0);//PITCH
yaw_trage_angle=YAW_START_ANGLE+720*(scan_percent_YAW/1000.0);//YAW��תһȦ��һ��
		}
		else //�Ӿ�����װ�װ�-ɨ�����
		{
			scan_time=0;
		if(Armour_lose_time>500)//���Ƶ�λ-ɨ�迪ʼ
YAW_START_ANGLE=DJIC_IMU.total_yaw;//˿����ʼɨ��
scan_percent_PITCH=	(DJIC_IMU.total_pitch-PITCH_MIN_angle)/allow_angle*1000	;	
scan_percent_YAW=0;	
		
		}

	}
	else//����ɨ�赵λ
	{
		scan_time=0;
		YAW_START_ANGLE=DJIC_IMU.total_yaw;//˿����ʼɨ��
		scan_percent_PITCH=	(DJIC_IMU.total_pitch-PITCH_MIN_angle)/allow_angle*1000	;	
scan_percent_YAW=0;
	}
		
	

	
	
}

