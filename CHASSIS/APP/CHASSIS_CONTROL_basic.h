#ifndef CHASSIS_basic_H
#define CHASSIS_basic_H

#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "MY_CHASSIS_CONTROL.h"

#if chassis_radom_mode_two ==1

void CHASSIS_CONTROUL_2(void);

//void just_arrive_targe_speed(int targe_speed);

void star_and_new(void);

typedef struct
{
int game_state_progress;/*0��δ��ʼ������
? 1��׼���׶Σ�
? 2���Լ�׶Σ�
? 3��5s ����ʱ��
? 4����ս�У�
? 5������������
	*/
int this_progress_remain_time;/*��ǰ�׶�ʣ��ʱ�䣬��λ */
	
bool red_outpost_is_live;
bool blue_outpost_is_live;	
bool our_outpost_is_live;
} CHASSIS_KEY;

extern CHASSIS_KEY key_message;

extern int send_to_chassis_JUST_MOVE;//���͸����̵�����_�պ��㹻������
extern int init_times;


#endif

#endif