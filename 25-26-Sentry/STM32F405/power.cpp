#include "power.h"
#include "motor.h"
#include "control.h"
#include "judgement.h"
#include "supercap.h"


float POWER::Power_Update()//�򵥰汾�����㹦������
{
	//��ȡ����ϵͳ�������
	consume_power = judgement.data.power_heat_data_t.chassis_power;
	powerlimitation = judgement.data.robot_status_t.chassis_power_limit;
	powerbuffer = judgement.data.power_heat_data_t.chassis_power_buffer;

	//������ù���
	float available = powerbuffer + powerlimitation - consume_power - safe_buffer;

	//���㹦�ʿ�������
	if (consume_power <= powerlimitation - safe_buffer)
	{
		power_coef = 1.f;
	}
	else if (consume_power <= powerlimitation)
	{
		power_coef = 1.0 - alpha * (consume_power - powerlimitation + safe_buffer) / safe_buffer;
	}
	else if (consume_power <= powerlimitation + powerbuffer - safe_buffer)
	{
		power_coef = (1.f - alpha) * (available / (powerbuffer - safe_buffer)) + 0.1;
	}
	else
	{
		power_coef = 0.1;
	}

	//�Թ��ʿ������ӽ����˲�
	power_coef = beta * pre_power_coef + (1.f - beta) * power_coef;

	//�ƺ���
	pre_power_coef = power_coef;

	return(power_coef);
}

void POWER::slopeupdate()
{
	if (powerlimitation <= 50)
	{
		rampslope = 4;
	}
	else if (powerlimitation <= 60)
	{
		rampslope = 6;
	}
	else if (powerlimitation <= 80)
	{
		rampslope = 8;
	}
	else
	{
		rampslope = 10;
	}
}

void POWER::fit()
{
	//��ȡ����ϵͳ�������
	consume_power = judgement.data.power_heat_data_t.chassis_power;
	powerlimitation = judgement.data.robot_status_t.chassis_power_limit;
	powerbuffer = judgement.data.power_heat_data_t.chassis_power_buffer;

	data4fit.rpm = ctrl.chassis_motor[0]->curspeed;
	data4fit.curcurrent = ctrl.chassis_motor[0]->curcurrent;
	data4fit.setcurrent = ctrl.chassis_motor[0]->setcurrent;
	data4fit.bus_voltage = 24.221;
	//data4fit.actual_current = supercap.supercap_union.supercap.I;

	//supercap.Control(consume_power, 1, powerbuffer);
}

float calc_koe(float current_error) // 
{
	// ������Χ
	float error_lower = 500.0;   // �趨����
	float error_upper = 9000.0; // �趨����

	// ���㵱ǰ���ֵ
	float error_i = current_error; // ���赱ǰ�����Ϊ current_error

	// ����kcoe��ֵ��ʹ�����Բ�ֵ����
	float kcoe;
	if (error_i <= error_lower) {
		kcoe = 0.0;  // ������С�ڻ����������kcoeΪ0
	}
	else if (error_i >= error_upper) {
		kcoe = 1.0;  // ��������ڻ����������kcoeΪ1
	}
	else {
		// ��Χ��ͨ�����Բ�ֵ����kcoe
		kcoe = (error_i - error_lower) / (error_upper - error_lower);
	}

	return kcoe;
}

float POWER::complementary_filter(float input, float pre_output)
{
	float output = (1 - alpha) * pre_output + alpha * input;//�����alpha��ʾ��ǰֵ��Ȩ�أ�
	return output;
}

void POWER::RCS_PowerUpdate()
{
	if (fly_flag)
	{
		for (int i = 0; i < 4; i++)
		{
			ctrl.chassis_motor[i]->powercoef = 1.f;
		}
		return;
	}

	uint16_t max_power_limit = 30;
	uint16_t power_buffer_remain = 0;

	float initial_give_power[4];
	float scaled_give_power[4];

	float chassis_power = 0.0f;
	float chassis_buffer = 0.0f;

	initial_total_power_pre = initial_total_power;

	//�����ĸ�����ԭ������������m3508��� �������Լ�����hkust��Դ�ķ������б궨
	float toque_coefficient = 1.99688994e-6f * 1.1;
	float k1 = 1.931e-07 / 8;
	float k2 = 1.381e-06 / 8;
	float constant = 4.54f / 8;

	//����ϵͳ������ر����Ķ�ȡ
	max_power_limit = judgement.data.robot_status_t.chassis_power_limit;
	power_buffer_remain = judgement.data.power_heat_data_t.chassis_power_buffer;

	//��ǰ����
	//chassis_power = judgement.data.power_heat_data_t.chassis_power; //25������Դ����ģ�鲻���ṩ������Ϣ����޷�ʹ��


	//�����ڳ���ʱ������ͨ�����ʰ�ʵ�ָ���ȷ�ĵ�ǰ���ʵĶ�ȡ
	if (supercap.connect)
	{
		//����ʣ�๦��ά����30w
		consume_power = supercap.Rxsuper.U * supercap.Rxsuper.I;
		if (consume_power < 0) consume_power = 0;

		power_remain = max_power_limit - consume_power + power_buffer_remain;

		input_power = power_controller[1].Position(power_remain - 30.f, 1000.f);
		if (input_power <= 0) input_power = 0;

		if (supercap.Rxsuper.cap_energy < 100.f)
		{
			//ȫ�Զ����� ֻ��Ҫ���������ڳ������
			chassis_max_power = input_power;
		}
		else
		{
			chassis_max_power = 120;
		}
	}
	else
	{
		//�����ڳ���ʱ��ͨ��pid������ά�ֻ�����Ϊһ����ֵ
		if (max_power_limit != 0.f) // ����60w�Ļ���������
		{
			input_power = max_power_limit + power_controller[0].Position(power_buffer_remain - 60.f, 1000.f);
			if (input_power <= 0) input_power = 0;
		}
		else // ����ϵͳ���� ��ʹ����С��������
		{
			input_power = max_power_limit;
		}

		chassis_max_power = input_power;
	}

	//���Ƶ�ǰ�������
	float tmp_total = 0.0f;
	float give_power_range = 80.f;
	for (uint8_t i = 0; i < 4; i++)
	{
		initial_give_power[i] = toque_coefficient * abs(ctrl.chassis_motor[i]->setcurrent * ctrl.chassis_motor[i]->curspeed) + \
			k2 * ctrl.chassis_motor[i]->curspeed * ctrl.chassis_motor[i]->curspeed + \
			k1 * ctrl.chassis_motor[i]->setcurrent * ctrl.chassis_motor[i]->setcurrent + \
			constant;

		if (initial_give_power[i] > give_power_range) initial_give_power[i] = give_power_range; //����ֵ���ܻᷢ������ͻ�� �ų�����ֵ

		tmp_total += initial_give_power[i];
	}
	initial_total_power = complementary_filter(tmp_total, initial_total_power_pre);

	//�����ط���
	// ÿ������ķֺ���/ÿ������ķ�ǰ���� = ����˥������
	//�������ٶȺ͵�����˵Ҫ�ø�����������ӣ�
	if (initial_total_power > chassis_max_power)
	{
		float power_scale = chassis_max_power / initial_total_power;

		// ����1��ƽӹ����
		for (int i = 0; i < 4; i++)
		{
			//scaled_give_power[i] = abs(initial_give_power[i] * power_scale); 
			ctrl.chassis_motor[i]->powercoef = fabs(power_scale); //sqrt(scaled_give_power[i] / initial_give_power[i]);
		}

		//����2���������ĵȱ�����
		//float sum_error = 0.0f;
		//float errors[4], k_coe[4];
		//for (int i = 0; i < 4; i++)
		//{
		//	errors[i] = abs(ctrl.chassis_motor[i]->setspeed - ctrl.chassis_motor[i]->curspeed);
		//	k_coe[i] = calc_koe(errors[i]);  
		//	sum_error += errors[i];  // �����ܺ�
		//}
		//for (int i = 0; i < 4; i++)
		//{
		//	// ����������Ĳ������书��
		//	float error_weight = errors[i] / sum_error;  // �����
		//	float cmd_power_weight = abs(initial_give_power[i] * power_scale);  // ������
		//	// �������չ���
		//	scaled_give_power[i] = k_coe[i] * chassis_max_power * error_weight + // û��chassis_max_power �ۿ�Ҳ����
		//		(1 - k_coe[i]) * cmd_power_weight;
		//	// ���µ���Ĺ���ϵ��
		//	ctrl.chassis_motor[i]->powercoef = sqrt(scaled_give_power[i] / initial_give_power[i]);
		//}

	}
	else // ���������ʿ���
	{
		for (int i = 0; i < 4; i++)
		{
			ctrl.chassis_motor[i]->powercoef = 1.f;
		}
	}
}