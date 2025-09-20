#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"
#include "pid.h"



class POWER
{
public:
	float consume_power, powerlimitation, powerbuffer, safe_buffer = 20.f;
	float power_coef, pre_power_coef, rampslope;
	float alpha = 0.9f, beta = 0.1f;//�������Ӳ������˲�����
	//���Ƶ����� alpha = 0.3f, beta = 0.1f
	//�����ٶȣ� alpha = 0.3f, beta = 0.1f

	float Power_Update();//�򵥰汾
	void slopeupdate();//����ramp��slope

	void fit();//���ڱ궨����ʱʹ��
	void RCS_PowerUpdate();
	float complementary_filter(float input, float pre_output);
	float initial_total_power, initial_total_power_pre = 0.f, input_power, chassis_max_power;
	float power_remain = 0.0f;
	PID power_controller[2];//0:��������������1�������������
	//�����Ƿ�ϵ����������ֵı�����
	float Pin, w, Icmd, Ct = 1.99688994e-6f, k1, k2;
	struct {
		float setcurrent;
		float curcurrent;
		float bus_voltage;
		float rpm;
		float actual_current;
	}data4fit;

	bool fly_flag = true;

private:

};

extern POWER power;