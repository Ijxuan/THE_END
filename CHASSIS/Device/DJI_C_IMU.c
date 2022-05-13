#include "DJI_C_IMU.h"
#include "GM6020.h"

#include "Vision.h"
#include "FPS_Calculate.h"

DR16_R_FROM_C R16_from_C;

//��������������
DJI_C_Euler_u DJI_C_Euler_Receive;
DJI_C_Gyro_u DJI_C_Gyro_Receive;
//֡�ʽṹ��
Frame_rate_t Euler_Framerate;
Frame_rate_t Gyro_Framerate;
//���ݴ���ṹ��
DJIC_IMU_t DJIC_IMU;

//���DJI_C��DR16
void DJI_C_DR16_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
//	if (CAN_Rx_Structure.CAN_RxMessage.StdId != DR16_R_ID)
//	{
//		return;
//	}
	for (int i = 0; i < 8; i++)
	{
		R16_from_C.BraodData[i] = CAN_Rx_Structure.CAN_RxMessageData[i];
	}
//	//��ȡ��֡��
//	Get_FPS(&FPS_ALL.IMU_Gyro.WorldTimes, &FPS_ALL.IMU_Gyro.FPS);
//		//���߼��
//	Gyro_Framerate.Offline_Detec++;
//	Gyro_Framerate.Offline_Flag = 0;
}


//�����������
void Updata_Hand_Euler_Gyro_Data(void)
{
	//�Ƕ�
	DJIC_IMU.yaw = (float)DJI_C_Euler_Receive.yaw * Angle_turn_Radian + 180.0f;		//������תΪ��
	DJIC_IMU.pitch = (float)DJI_C_Euler_Receive.pitch * Angle_turn_Radian + 180.0f; //(-180�� ~ 180��)
//	Vision_Cloud.VisionSend_t.YawAngle_Error=DJIC_IMU.yaw;
//	Vision_Cloud.VisionSend_t.PitchAngle_Error=DJIC_IMU.pitch;
	//���ٶ�
	DJIC_IMU.Gyro_z = DJI_C_Gyro_Receive.Gyro_z * Angle_turn_Radian;
	DJIC_IMU.Gyro_y = DJI_C_Gyro_Receive.Gyro_y * Angle_turn_Radian;
//	Vision_Cloud.VisionSend_t.YawAngle_Error=DJIC_IMU.Gyro_z;
//	Vision_Cloud.VisionSend_t.PitchAngle_Error=DJIC_IMU.Gyro_y;

	if (DJIC_IMU.yaw - DJIC_IMU.last_yaw < -300.0f)
	{
		DJIC_IMU.yaw_turnCounts++;
	}
	if (DJIC_IMU.last_yaw - DJIC_IMU.yaw < -300.0f)
	{
		DJIC_IMU.yaw_turnCounts--;
	}
	DJIC_IMU.total_yaw = DJIC_IMU.yaw + DJIC_IMU.yaw_turnCounts * 360.0f;
	DJIC_IMU.last_yaw = DJIC_IMU.yaw;

	//Pitch
	if (DJIC_IMU.pitch - DJIC_IMU.last_pitch < -300.0f)
	{
		DJIC_IMU.pitch_turnCounts++;
	}
	if (DJIC_IMU.last_pitch - DJIC_IMU.pitch < -300.0f)
	{
		DJIC_IMU.pitch_turnCounts--;
	}
	DJIC_IMU.total_pitch = DJIC_IMU.pitch + DJIC_IMU.pitch_turnCounts * 360.0f;
	DJIC_IMU.last_pitch = DJIC_IMU.pitch;
	
	
	Vision_Cloud.VisionSend_t.YawAngle_Error=DJIC_IMU.total_yaw;
	Vision_Cloud.VisionSend_t.PitchAngle_Error=DJIC_IMU.total_pitch;

}

//���DJI_C_IMU��Euler
void DJI_C_Euler_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if (CAN_Rx_Structure.CAN_RxMessage.StdId != DJI_C_Angle)
	{
		return;
	}
	for (int i = 0; i < 8; i++)
	{
		DJI_C_Euler_Receive.BraodData[i] = CAN_Rx_Structure.CAN_RxMessageData[i];
	}
	//��ȡ��֡��
	Get_FPS(&FPS_ALL.IMU_angle.WorldTimes, &FPS_ALL.IMU_angle.FPS);
	//���߼��
	Euler_Framerate.Offline_Detec++;
	Euler_Framerate.Offline_Flag = 0;
}

//���DJI_C_IMU��Gyro
void DJI_C_Gyro_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if (CAN_Rx_Structure.CAN_RxMessage.StdId != DJI_C_Gyro)
	{
		return;
	}
	for (int i = 0; i < 8; i++)
	{
		DJI_C_Gyro_Receive.BraodData[i] = CAN_Rx_Structure.CAN_RxMessageData[i];
	}
	//��ȡ��֡��
	Get_FPS(&FPS_ALL.IMU_Gyro.WorldTimes, &FPS_ALL.IMU_Gyro.FPS);
		//���߼��
	Gyro_Framerate.Offline_Detec++;
	Gyro_Framerate.Offline_Flag = 0;
}

void imu_angle()
{
	PITCH_MAX_angle=DJIC_IMU.total_pitch+(7130-GM6020s[1].totalAngle)/8196.0*360.0;
	PITCH_MIN_angle=DJIC_IMU.total_pitch+(6430-GM6020s[1].totalAngle)/8196.0*360.0;
}


