#include "MY_CLOUD_CONTROL.h"
#include "FPS_Calculate.h"

#include "User_math.h"

Ramp_Struct *EM_Ramp;
void cloud_control(void)
{
	
								if(DR16.rc.s_left==1)//YAW����Ƶ�λ
							{
//							yaw_trage_angle+=(DR16.rc.ch0/660.0)/-3;//YAW��ң��������
//								PITCH_trage_angle+=(DR16.rc.ch1/660.0)*5;//pitch��ң��������
								
//							if(DR16.rc.ch4_DW<=-400)//����
//							yaw_trage_angle=yaw_trage_angle2;//�����ǽ��ٶ����Ϊ140
							/*
							����Ŀ���ԾҪ����140/0.03   4666
							//��Ҫ����29000�������ò��ܵ�140�������ٶ�
							//�ȸ�10000�Ľ�Ծ��
							������800
*/
//							if(DR16.rc.ch4_DW>=400)//����
//							yaw_trage_angle=0;
							}
/*					else
					��-10000��30000���ʼ20000���������ٶΣ�Ȼ����10000��220�������10000�ļ���		
					yaw_trage_angle+=(DR16.rc.ch3*1.0/660.0)*20;

*/
	
							
							YAW_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	

						
							PITCH_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void YAW_PID()
{
	//							PITCH_trage_angle=0;//����ˮƽλ��


	
	
	
	
	
						#if PID_MOTOR//YAW�����Ƕ�
					P_PID_bate(&Yaw_Angle_pid, yaw_trage_angle,GM6020s[0].totalAngle);//GM6020s[EMID].totalAngle readAngle

					yaw_trage_speed=Yaw_Angle_pid.result;//��Ծ


					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
					P_PID_bate(&Yaw_Speed_pid, yaw_trage_speed,GM6020s[0].readSpeed);
					#endif
#if PID_YAW_EM==1	  //�Ƿ���YAW����������  1����

	
				if(DR16.rc.s_right==2)	//�Ƿ�������
				{		
#endif	
					#if PID_IMU//YAW��������

//					Yaw_IMU_Angle_pid.Kp=-YAW_IMU_Kp;//���Թ��������ֵҪ���ϸ���
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
//					send_to_pitch=(DR16.rc.ch3*1.0/660.0)*29000;
					#endif
#if PID_YAW_EM==1	  //�Ƿ���YAW����������  1����
					
				}
				else//�Ӿ�����
				{
				if(FPS_ALL.Vision.FPS>20&&VisionData.RawData.Armour == 1)	
										{
											yaw_trage_angle=DJIC_IMU.total_yaw;
//											EM_Ramp->Target_Value=VisionData.RawData.Yaw_Angle;
//											EM_Ramp->Current_Value=DJIC_IMU.total_yaw;
//											EM_Ramp->Absolute_Max=50;
yaw_trage_angle2=+VisionData.RawData.Yaw_Angle;
P_PID_bate(&Yaw_EM_Angle_pid, yaw_trage_angle2,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle

					yaw_trage_speed=Yaw_EM_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
					
P_PID_bate(&Yaw_EM_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
					
					send_to_yaw=Yaw_EM_Speed_pid.result;
										}
				else
				{
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
					
				}
				
				}
#endif	

	
	
}

void PITCH_PID()
{
												if(DR16.rc.s_right==3)	//�Ƿ�������
				{		
							if(FPS_ALL.Vision.FPS>20&&VisionData.RawData.Armour == 1)	
							{
								PITCH_trage_angle-=(Vision_RawData_Pitch_Angle/10.0);
//							PITCH_trage_angle_2=;
							}
							else
								PITCH_trage_angle =DJIC_IMU.total_pitch;
				}			
//							else
//								PITCH_trage_angle =PITCH_trage_angle_2;
							imu_angle();//ȥ���������ԣ�����  ����
							//��������Ư�޷�
						if(PITCH_trage_angle>PITCH_MAX_angle||PITCH_trage_angle==PITCH_MAX_angle)
													PITCH_trage_angle=PITCH_MAX_angle;
						if(PITCH_trage_angle<PITCH_MIN_angle||PITCH_trage_angle==PITCH_MIN_angle)
													PITCH_trage_angle=PITCH_MIN_angle;	
	
	
	
	#if PID_PITCH_MOTOR      //PITCH�����Ƕ�
if(PITCH_trage_angle>7125)
	PITCH_trage_angle=7125;
if(PITCH_trage_angle<6435)
	PITCH_trage_angle=6435;
					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle,GM6020s[1].totalAngle);//GM6020s[EMID].totalAngle readAngle
//�����ǵ��ٶ�ֵ����С��
					PITCH_trage_speed=PITCH_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
				
				P_PID_bate(&PITCH_Speed_pid, PITCH_trage_speed,GM6020s[1].readSpeed);
				send_to_pitch=PITCH_Speed_pid.result;
#endif	
	

#if PID_PITCH_EM==1	  //�Ƿ���PITCH����������  1����
	
					if(DR16.rc.s_right==2)	//�Ƿ�������
				{		
#endif
					
	#if PID_PITCH_IMU		//PITCH��������
//							if(DR16.rc.s_left==3)//PITCH����Ƶ�λ
//							{
//							PITCH_trage_angle+=(DR16.rc.ch3*1.0/660.0)*1;//ң�������ٶ�Ŀ��ֵ ��ѡһ

//							if(DR16.rc.ch4_DW<=-400)//����
//							PITCH_trage_angle=PITCH_MAX_angle-1;
//							if(DR16.rc.ch4_DW>=400)//����
//							PITCH_trage_angle=PITCH_MIN_angle+1;
//							
//							}

					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//�����ǵ��ٶ�ֵ����С��
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//��ȥ��ʵ��
#endif

#if PID_PITCH_EM==1	//�Ƿ���PITCH����������  1����
				}
				else//�Ӿ�����
				{
								if(FPS_ALL.Vision.FPS>20&&VisionData.RawData.Armour == 1)	
										{
					
//P_PID_bate(&PITCH_EM_Angle_pid,PITCH_trage_angle ,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle

//					PITCH_trage_speed=PITCH_EM_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					
//P_PID_bate(&PITCH_EM_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
//					
//					send_to_pitch=PITCH_EM_Speed_pid.result;
					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//�����ǵ��ٶ�ֵ����С��
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//��ȥ��ʵ��
										}
				else
				{
					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//�����ǵ��ٶ�ֵ����С��
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//��ȥ��ʵ��
					
				}
										
										
				}
#endif	

}


