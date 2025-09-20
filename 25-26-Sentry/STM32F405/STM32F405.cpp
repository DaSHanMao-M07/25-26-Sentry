  /*
   *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\\\____________/\\\\\\\\\_____/\\\\\\\\\\\___
   * _\///\\\___/\\\/__\/\\\\\\________/\\\\\\_\/\\\_______\/\\\____________/\\\///////\\\_______/\\\////////____/\\\/////////\\\_
   *  ___\///\\\\\\/____\/\\\//\\\____/\\\//\\\_\/\\\_______\/\\\___________\/\\\_____\/\\\_____/\\\/____________\//\\\______\///__
   *   _____\//\\\\______\/\\\\///\\\/\\\/_\/\\\_\/\\\_______\/\\\___________\/\\\\\\\\\\\/_____/\\\_______________\////\\\_________
   *    ______\/\\\\______\/\\\__\///\\\/___\/\\\_\/\\\_______\/\\\___________\/\\\//////\\\____\/\\\__________________\////\\\______
   *     ______/\\\\\\_____\/\\\____\///_____\/\\\_\/\\\_______\/\\\___________\/\\\____\//\\\___\//\\\____________________\////\\\___
   *      ____/\\\////\\\___\/\\\_____________\/\\\_\//\\\______/\\\____________\/\\\_____\//\\\___\///\\\___________/\\\______\//\\\__
   *       __/\\\/___\///\\\_\/\\\_____________\/\\\__\///\\\\\\\\\/_____________\/\\\______\//\\\____\////\\\\\\\\\_\///\\\\\\\\\\\/___
   *        _\///_______\///__\///_____________\///_____\/////////_______________\///________\///________\/////////____\///////////_____
  */

//2024/6/1 ͯҢ ������
//2025/7/21 ��ɽè

//��ݼ���
//	CTRL+��� ת������
//	CTRL+SHIFT+/ ע��ѡ������
//	CTRL+R+R �޸ı�����

//�����淶��
//			������Сд��ĸ���»��߷ָ�ʣ��� pre_yaw
//			���������շ������������˵�һ�����ʣ�������������ĸ��д���� keepGimbal()
//�����в����Ϲ淶��������Ŀ�ݼ���һ��


#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "XUC.h"

#include "DMmotor.h"
#include "supercap.h"
#include "power.h"

Motor can1_motor[CAN1_MOTOR_NUM] =//��̨
{
	Motor(M3508,SPD,shooter, ID1, PID(0.0f, 0.f, 0.0f)),
	Motor(M3508,SPD,shooter, ID2, PID(1.78f, 0.f, 0.0f)),
	Motor(M3508,SPD,shooter, ID3, PID(1.78f, 0.f, 0.0f)),
	Motor(M3508,SPD,shooter, ID4, PID(0.0f, 0.f, 0.0f)),//Ħ����
	Motor(M2006,SPD,supply,  ID7, PID(1.8f, 0.f, 0.4f)),
	Motor(M2006,ACE,supply,  ID8, PID(0.8f, 0.f, 1.f)),//������

	Motor(M6020,AUTO_Y,gimbal, ID5, PID(32.88f, 0.06f, 150.5f,0.f),//YAW
								 PID(1.38f, 0.06f, 30.5f,0.f),  //�ֲ� PID ����
								 PID(26.8f, 0.01f, 20.f,0.f),	//62.5 30 0.02 25
								 PID(0.68f, 0.01f, 10.f,0.f),  //���� PID ���� 1.88 0.12 15
								 PID(0.f, 0.f, 0.f,0.f),//����λ
								     0.f),//ǰ������ϵ�� 1.5
	Motor(M6020,TEST,gimbal, ID6, PID(80.0f, 0.f, 50.f,0.f),//PITCH
								 PID(0.87f, 0.f, 12.5f,0.f),  //�ֲ� PID ����  
								 PID(84.88f, 0.15f, 111.22f,0.f),
								 PID(0.79f, 0.11f, 21.11f,0.f),  //���� PID ����
								     0.35f),//ǰ������ϵ��
									 //50 0.1 100
									 //0.7 0.1 20

	//��һ�����ٶȻ����ڶ�����λ�û���������Ҫ��λ�û�����
};

Motor can2_motor[CAN2_MOTOR_NUM] =//�ֵ�� ��yaw��
{
	Motor(M3508,SPD,chassis, ID1, PID(3.5f, 0.f, 0.05f)),
	Motor(M3508,SPD,chassis, ID2, PID(3.5f, 0.f, 0.05f)),
	Motor(M3508,SPD,chassis, ID3, PID(3.5f, 0.f, 0.05f)),
	Motor(M3508,SPD,chassis, ID4, PID(3.5f, 0.f, 0.05f)),
};

DMMOTOR DM_motorYaw = { DMMOTOR(0x06 , &can2) };
DMMOTOR DM_motorPitch = { DMMOTOR(0x09 , &can1) };


CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_gimbalS;
IMU imu_gimbalL;
DELAY delay;
RC rc;
XUC xuc;
LED led1, led2;
TASK task;
CONTROL ctrl;
Judgement judgement;
SUPERCAP supercap;
POWER power;


int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();
	can1.Init(CAN1);
	can2.Init(CAN2);	
	timer.Init(BASE, TIM3, 1000).BaseInit();

	judgement.Init(&uart3, 115200, USART3);
	xuc.Init(&uart1, USART1, 115200);
	imu_gimbalS.Init(&uart2, USART2, 115200, CH010);
	imu_gimbalL.Init(&uart4, UART4, 115200, CH010);
	rc.Init(&uart5, UART5, 100000);
	//supercap.Init(&uart1, 115200, USART1);

	ctrl.Init(//��ʵ��ƥ�� ����Խ��
		{ &can1_motor[0], &can1_motor[1], &can1_motor[2], &can1_motor[3],//Ħ����
	 	  &can1_motor[4], &can1_motor[5],//��̨
		  &can1_motor[6], &can1_motor[7],//������
		  &can2_motor[0], &can2_motor[1], &can2_motor[2], &can2_motor[3]//���̵��
		});

	task.Init();

	HAL_Delay(500);//�ȴ���ʼ�����

	DM_motorYaw.MotorStart(0x206);//DM yaw������ʼʹ��
	DM_motorPitch.MotorStart(0x109);//DM pitch������ʼʹ��

	for (;;)
		;
}

