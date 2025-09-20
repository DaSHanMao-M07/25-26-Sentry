#pragma once
#include <cinttypes>
#include <cstring>
#include <cmath>
#include"label.h"
#include "can.h"
#include "motor.h"
#include "kalman.h"

#define P_MIN -3.141593    // Radians
#define P_MAX 3.141593
#define P_MIN1 -12.56637    // Radians
#define P_MAX1 12.56637
#define V_MIN -45    // Rad/s
#define V_MAX 45
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define C_MAX 40.f		//����
#define C_MIN -40.f
#define T_MIN -20.0f    //����
#define T_MAX 20.0f
#define KT 1.4f
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

enum ERR { overVoltage = 8, underVoltage, overCurrent, mosOverT, motorOverT, missConnect, overLoad };



class DMMOTOR
{
public:
	uint32_t ID;
	uint32_t DM_ID_Yaw = 0x06;
	uint32_t DM_ID_Pitch = 0x09;
	//function_type function;
	ERR error;
	DMMOTOR(const uint32_t ID,  CAN* hcan);//����ӿ�
	void DMmotorOntimer(uint8_t idata[][8], uint8_t* odata); //�������ݲ����� ������ݲ�����
	void DMmotorTransmit(uint32_t id);//��������
	void MotorStart(uint32_t id);//���ʹ��
	void MotorOff(uint32_t id);//���ʧ��
	void MotorZero(uint32_t id);//���������
	void setDMmotor(float delta_pos, uint32_t id);//���ƺ���
	float setRange(const float original, const float range);

	bool initial = false;
	bool motorDisablity = false;
	bool flag_zero = false;
	bool flag_first = true;
	PID DM_pid { 4.5f,0.f,2.5f,0.f,1000.f};//DM��� �ٶ�ģʽ�� ��λ�õ��ٶȻ�PID
	PID DM_IMU_pid { 0.f,0.f,0.f,0.f,0.f };
	PID DM_IMU_ROTA_pid { 30.0f,0.0f,0.0f, 20.f, 20.f };
	PID DM_AUTO_pid{ 600.f,0.f,0.f,0.f,0.f };
	Kalman DM_kalman{ 1.f,40.f };

	float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
	}

	uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
	{
		float span = x_max - x_min;
		float offset = x_min;

		return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
	}


	CAN* mcan;

	float angle[2]{};//��Ϊ�ǰ�װ���ʱ����ʼ��̬�ĵ�������˼нǣ�
	float pos;//ת��λ�ã�Ҳ�Ƿ��������ĽǶ�
	float cur_speed;//�����ǰ�ٶ�
	float temp_ROTOR, temp_MOS;//����ڲ���Ȧ�¶� ���������MOS�¶�
	float set_speed = 0.f;//�����ٶ�
	float set_yaw_pos;//����������Ŀ��ֵ �������
	float set_pitch_pos, set_pitch_speed = 6.66f;
	float target_pos_yaw, target_pos_pitch;
	float initial_pos_yaw = 0.f, initial_pos_pitch=-0.75f;
	float DM_pitch_min = -1.15, DM_pitch_max = -0.2;
	float rota_speed = 5.f, nomal_speed = 0.f;
	//float current, setCurrent;//����
	//float torque, setTorque;//Ť��
private:
};

extern DMMOTOR DM_motorYaw;
extern DMMOTOR DM_motorPitch;