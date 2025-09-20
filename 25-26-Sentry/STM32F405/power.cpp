#include "power.h"
#include "motor.h"
#include "control.h"
#include "judgement.h"
#include "supercap.h"


float POWER::Power_Update()//简单版本：计算功率因子
{
	//读取裁判系统相关数据
	consume_power = judgement.data.power_heat_data_t.chassis_power;
	powerlimitation = judgement.data.robot_status_t.chassis_power_limit;
	powerbuffer = judgement.data.power_heat_data_t.chassis_power_buffer;

	//计算可用功率
	float available = powerbuffer + powerlimitation - consume_power - safe_buffer;

	//计算功率控制因子
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

	//对功率控制因子进行滤波
	power_coef = beta * pre_power_coef + (1.f - beta) * power_coef;

	//善后工作
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
	//读取裁判系统相关数据
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
	// 定义误差范围
	float error_lower = 500.0;   // 设定下限
	float error_upper = 9000.0; // 设定上限

	// 计算当前误差值
	float error_i = current_error; // 假设当前的误差为 current_error

	// 计算kcoe的值，使用线性插值方法
	float kcoe;
	if (error_i <= error_lower) {
		kcoe = 0.0;  // 如果误差小于或等于下限则kcoe为0
	}
	else if (error_i >= error_upper) {
		kcoe = 1.0;  // 如果误差大于或等于上限则kcoe为1
	}
	else {
		// 误差范围内通过线性插值计算kcoe
		kcoe = (error_i - error_lower) / (error_upper - error_lower);
	}

	return kcoe;
}

float POWER::complementary_filter(float input, float pre_output)
{
	float output = (1 - alpha) * pre_output + alpha * input;//类变量alpha表示当前值的权重！
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

	//以下四个参数原理上适用所有m3508电机 但建议自己按照hkust开源的方法进行标定
	float toque_coefficient = 1.99688994e-6f * 1.1;
	float k1 = 1.931e-07 / 8;
	float k2 = 1.381e-06 / 8;
	float constant = 4.54f / 8;

	//裁判系统功率相关变量的读取
	max_power_limit = judgement.data.robot_status_t.chassis_power_limit;
	power_buffer_remain = judgement.data.power_heat_data_t.chassis_power_buffer;

	//当前功率
	//chassis_power = judgement.data.power_heat_data_t.chassis_power; //25赛季电源管理模块不再提供功率信息因此无法使用


	//当存在超电时，可以通过功率板实现更精确的当前功率的读取
	if (supercap.connect)
	{
		//将总剩余功率维持在30w
		consume_power = supercap.Rxsuper.U * supercap.Rxsuper.I;
		if (consume_power < 0) consume_power = 0;

		power_remain = max_power_limit - consume_power + power_buffer_remain;

		input_power = power_controller[1].Position(power_remain - 30.f, 1000.f);
		if (input_power <= 0) input_power = 0;

		if (supercap.Rxsuper.cap_energy < 100.f)
		{
			//全自动超电 只需要处理超电正在充电的情况
			chassis_max_power = input_power;
		}
		else
		{
			chassis_max_power = 120;
		}
	}
	else
	{
		//不存在超电时：通过pid控制器维持缓冲区为一个定值
		if (max_power_limit != 0.f) // 保持60w的缓冲区能量
		{
			input_power = max_power_limit + power_controller[0].Position(power_buffer_remain - 60.f, 1000.f);
			if (input_power <= 0) input_power = 0;
		}
		else // 裁判系统离线 则使用最小功率输入
		{
			input_power = max_power_limit;
		}

		chassis_max_power = input_power;
	}

	//估计当前输出功率
	float tmp_total = 0.0f;
	float give_power_range = 80.f;
	for (uint8_t i = 0; i < 4; i++)
	{
		initial_give_power[i] = toque_coefficient * abs(ctrl.chassis_motor[i]->setcurrent * ctrl.chassis_motor[i]->curspeed) + \
			k2 * ctrl.chassis_motor[i]->curspeed * ctrl.chassis_motor[i]->curspeed + \
			k1 * ctrl.chassis_motor[i]->setcurrent * ctrl.chassis_motor[i]->setcurrent + \
			constant;

		if (initial_give_power[i] > give_power_range) initial_give_power[i] = give_power_range; //估计值可能会发生错误突变 排除错误值

		tmp_total += initial_give_power[i];
	}
	initial_total_power = complementary_filter(tmp_total, initial_total_power_pre);

	//功率重分配
	// 每个电机的分后功率/每个电机的分前功率 = 功率衰减因子
	//（对于速度和电流来说要用根号下这个因子）
	if (initial_total_power > chassis_max_power)
	{
		float power_scale = chassis_max_power / initial_total_power;

		// 方法1：平庸缩放
		for (int i = 0; i < 4; i++)
		{
			//scaled_give_power[i] = abs(initial_give_power[i] * power_scale); 
			ctrl.chassis_motor[i]->powercoef = fabs(power_scale); //sqrt(scaled_give_power[i] / initial_give_power[i]);
		}

		//方法2：基于误差的等比缩放
		//float sum_error = 0.0f;
		//float errors[4], k_coe[4];
		//for (int i = 0; i < 4; i++)
		//{
		//	errors[i] = abs(ctrl.chassis_motor[i]->setspeed - ctrl.chassis_motor[i]->curspeed);
		//	k_coe[i] = calc_koe(errors[i]);  
		//	sum_error += errors[i];  // 误差的总和
		//}
		//for (int i = 0; i < 4; i++)
		//{
		//	// 计算基于误差的补偿分配功率
		//	float error_weight = errors[i] / sum_error;  // 误差项
		//	float cmd_power_weight = abs(initial_give_power[i] * power_scale);  // 功率项
		//	// 计算最终功率
		//	scaled_give_power[i] = k_coe[i] * chassis_max_power * error_weight + // 没乘chassis_max_power 港科也干了
		//		(1 - k_coe[i]) * cmd_power_weight;
		//	// 更新电机的功率系数
		//	ctrl.chassis_motor[i]->powercoef = sqrt(scaled_give_power[i] / initial_give_power[i]);
		//}

	}
	else // 不触发功率控制
	{
		for (int i = 0; i < 4; i++)
		{
			ctrl.chassis_motor[i]->powercoef = 1.f;
		}
	}
}