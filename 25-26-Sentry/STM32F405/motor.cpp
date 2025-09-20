#include "motor.h"
#include "gpio.h"
#include "control.h"
#include "XUC.h"
#include "RC.h"

uint32_t theta = 200;

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _autospeed, PID _autoposition, PID _rotation_mode, float _Kff_auto)//YAW

	: ID(id)
	, type(type)
	, mode(mode)
{
	getMax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[autospeed], &_autospeed, sizeof(PID));
	memcpy(&pid[autoposition], &_autoposition, sizeof(PID));
	memcpy(&pid[rotation_mode], &_rotation_mode, sizeof(PID));
	memcpy(&Kff_autoYaw, &_Kff_auto, sizeof(float));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _autospeed, PID _autoposition, float _Kff_auto)//PITCH

	: ID(id)
	, type(type)
	, mode(mode)
{
	getMax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[autospeed], &_autospeed, sizeof(PID));
	memcpy(&pid[autoposition], &_autoposition, sizeof(PID));
	memcpy(&Kff_autoPitch, &_Kff_auto, sizeof(float));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position) //POS模式
	: ID(id)
	, type(type)
	, mode(mode)
{
	getMax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed) //SPD模式
	: ID(id)
	, type(type)
	, mode(mode)
{
	getMax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	this->function = function;
}

float Motor::FF_Auto(float _delta, uint32_t _ID) //自瞄 该前馈控制器将前馈控制量近似看作与目标变化量成线性关系
{
	if (_ID == 4)//YAW轴电机ID
	{
		return (Kff_autoYaw * _delta);
	}
	else if (_ID == 5)//PITCH轴电机ID
	{
		return (Kff_autoPitch * _delta);
	}
}

float Motor::FF_Uphill()
{

}

float Motor::FF_Friction()
{

}

void Motor::statusIdentifier(int32_t torque_current)
{
	if (torque_current == old_torque_current)
		disconnect_count++;
	else
		disconnect_count = 0;

	if (disconnect_count >= disconnect_max)
	{
		disconnect_count = disconnect_max;
		if (old_torque_current == 0)
			m_status = UNCONNECTED;
		else
			m_status = DISCONNECTED;
	}
	else
		m_status = FINE;

	old_torque_current = torque_current;
}
uint8_t Motor::getStatus()const
{
	return (uint8_t)m_status;
}
void Motor::onTimer(uint8_t idata[][8], uint8_t* odata)//idate: receive;odate: trainsmit;RC
{
	IDx = this->ID - ID1;
	angle[now] = getWord(idata[IDx][0], idata[IDx][1]);
	curspeed = getWord(idata[IDx][2], idata[IDx][3]);
	curcurrent = getWord(idata[IDx][4], idata[IDx][5]);
	temperature = idata[IDx][6];
	this->statusIdentifier(this->curcurrent);
	if (this->m_status == FINE)
	{
		if (mode == ACE)
		{
			int rollspeed = 2000;//转动速度 2750

			if (need_curcircle > 0)
			{
				if (stopAngle > 4096)
					stopAngle -= 4096;
				setspeed = rollspeed;
				current += pid[speed].Delta(setspeed - curspeed);
				current = setRange(current, max_current);;
				if (count && (angle[now] < stopAngle || angle[now] > 4096 + stopAngle))
				{
					need_curcircle--;
					cur_circle++;
					count = 0;
				}
				else if (angle[now] > stopAngle && angle[now] < 4096 + stopAngle)
				{
					count = 1;
				}
				if (need_curcircle <= 0)
				{
					need_curcircle = 0;
					stopAngle = angle[now];
					pd = 0;
					setspeed = 0;
					current = 0;
					count = 0;
				}
			}
			///
			else if (need_curcircle == 0)
			{
				setspeed = 0;
				current += pid[speed].Delta(setspeed - curspeed);
				current = setRange(current, max_current);
				stopAngle = angle[now];
				count = 0;
			}
			///
			else if (need_curcircle < 0)
			{
				if (stopAngle > 4096)
					stopAngle -= 4096;
				setspeed = -1 * rollspeed;
				current += pid[speed].Delta(setspeed - curspeed);
				current = setRange(current, max_current);
				if (count && angle[now] > stopAngle && angle[now] < 4096 + stopAngle)
				{
					need_curcircle++;
					cur_circle--;
					count = 0;
				}
				else if (angle[now] < stopAngle || angle[now] > 4096 + stopAngle)
				{
					count = 1;
				}
				if (need_curcircle >= 0)
				{
					need_curcircle = 0;
					stopAngle = angle[now];
					pd = 0;
					setspeed = 0;
					current = 0;
					count = 0;
				}

			}
			setcurrent = current;

		}
		else if (mode == SPD)
		{
			if (function == chassis)
			{
				if (ctrl.mode == CONTROL::MODE::RESET)
				{
					setcurrent = 0;
					current = 0;
					setspeed = 0;
				}
				else if (!judgement.data.robot_status_t.power_management_chassis_output)
				{
					setcurrent = 0;
					current = 0;
				}
				else
				{
					setspeed = setRange(setspeed, 10000);
					current += pid[speed].Delta(setspeed - curspeed);
					setcurrent = setRange(current, max_current);
				}
			}
			else if (function == shooter)
			{
				if (ctrl.mode == CONTROL::MODE::RESET)
				{
					setspeed = 0.f;
					current += pid[speed].Delta(setspeed - curspeed);
					if (fabs(setspeed - curspeed)<=50.f)
					{
						current = 0.f;
					}
					setcurrent = setRange(current, 3000.f);
				}
				else
				{
					setspeed = setRange(setspeed, 10000.f);
					current += pid[speed].Delta(setspeed - curspeed);
					if (fabs(setspeed - curspeed) <= 20.f)
					{
						current = 0.f;
					}
					setcurrent = setRange(current, 3000.f);
				}
			}
			else if (function == supply)
			{
				setspeed = setRange(setspeed, 10000);
				current += pid[speed].Delta(setspeed - curspeed);
				setcurrent = setRange(current, max_current);
			}
			
		}
		else if (mode == SPDC)//底盘轮电机 功率控制
		{
			float tmpsetspeed = powercoef * setspeed;
			current += pid[speed].Delta(tmpsetspeed - curspeed);
			current = setRange(current, max_current);
			setcurrent = current * powercoef;
		}
		else if (mode == POS)
		{
			if (ctrl.mode == CONTROL::AUTOMATION)
			{
				setspeed = pid[autoposition].Position(kalman.Filter(getDelta(set_angle - angle[now])), 500.f);
				setspeed = setRange(setspeed, max_speed);
				current = pid[autospeed].Position(setspeed - curspeed, 1000.f);
				current = current_kalman.Filter(current);
				setcurrent = setRange(current, max_current);
			}
			else
			{
				setspeed = pid[position].Position(kalman.Filter(getDelta(set_angle - angle[now])), 500.f);
				setspeed = setRange(setspeed, max_speed);
				current = pid[speed].Position(setspeed - curspeed, 1000.f);
				current = current_kalman.Filter(current);
				setcurrent = setRange(current, max_current);
			}


		}

		else if (mode == AUTO_Y)//小云台YAW轴
		{
			err = kalman.Filter(getDelta(set_angle - angle[now])) / 8192 * 360;
			if (ctrl.mode == CONTROL::AUTOMATION )//自瞄
			{
				float angle_pid;
				float delta_target = getDelta(set_angle - pre_set_angle);

				//15 7.5 22.5
				feedforward = FF_Auto(delta_target, IDx);
				feedforward = setRange(feedforward, 150.f);//计算前馈量并限幅

				angle_pid = pid[autoposition].Position(kalman.Filter(getDelta(set_angle - angle[now])) - delta_target, 75.f);
				setspeed = angle_pid + feedforward;//角度——速度环PID控制器+前馈控制器
				setspeed = setRange(setspeed, 225.f);//限幅

				current = pid[autospeed].Position(setspeed - curspeed, 1000.f);//速度——电流环
				current = current_kalman.Filter(current);
				setcurrent = setRange(current, max_current);

				//if (fabs(xuc.velocity_w) < 0.5)
				//{
				//	angle_pid = static_pos.Position(kalman.Filter(getDelta(set_angle - angle[now])) - delta_target, 5.f);
				//	setspeed = angle_pid + feedforward;//角度——速度环PID控制器+前馈控制器
				//	setspeed = setRange(setspeed, 15.f);//限幅

				//	current = static_speed.Position(setspeed - curspeed, 2000.f);//速度——电流环
				//	current = current_kalman.Filter(current);
				//	setcurrent = setRange(current, max_current);
				//}

				pre_set_angle = set_angle; //更新目标角度历史值
			}
			else
			{
				setspeed = pid[position].Position(kalman.Filter(getDelta(set_angle - angle[now])), 500.f);
				setspeed = setRange(setspeed, max_speed);
				current = pid[speed].Position(setspeed - curspeed, 1000.f);
				current = current_kalman.Filter(current);
				setcurrent = setRange(current, max_current);
			}

		}

		else if (mode == AUTO_P)//小云台PITCH轴
		{
			if (ctrl.mode == CONTROL::AUTOMATION )//自瞄
			{
				setspeed = pid[autoposition].Position(kalman.Filter(getDelta(set_angle - angle[now])), 500.f);
				setspeed = setRange(setspeed, max_speed);
				current = pid[autospeed].Position(setspeed - curspeed, 1000.f);
				current = current_kalman.Filter(current);
				setcurrent = setRange(current, max_current);

			}
			else
			{
				float gravity_feedforward;//为电流值（即扭矩 二者为倍数关系）由公式 Tq=mgrcos(β+γ) 得到
				setspeed = pid[position].Position(kalman.Filter(getDelta(set_angle - angle[now])), 500.f);
				setspeed = setRange(setspeed, max_speed);
				current = pid[speed].Position(setspeed - curspeed, 1000.f);
				current = current_kalman.Filter(current) + gravity_feedforward;
				setcurrent = setRange(current, max_current);
			}

		}
		else if (mode == TEST)
		{
			current = test_torque;
			setcurrent= setRange(current, max_current);
		}
	}

	if (this->m_status != FINE)
	{
		setcurrent = 0;
		current = 0;
	}

	
	angle[pre] = angle[now];           //更新角度
	odata[IDx * 2] = (setcurrent & 0xff00) >> 8;
	odata[IDx * 2 + 1] = setcurrent & 0x00ff;      //向电调发送报文
	
}

void Motor::getMax(const type_t type)
{
	adjspeed = 3000.f;
	switch (type)
	{
	case M3508:
		max_current = 6000.f;
		max_speed = 3800.f;
		break;
	case M3510:
		max_current = 13000.f;
		max_speed = 9000.f;
		break;
	case M2310:
		max_current = 13000.f;
		max_speed = 9000.f;
		adjspeed = 1000.f;
		break;
	case EC60:
		max_current = 5000.f;
		max_speed = 300.f;
		break;
	case M6623:
		max_current = 5000.f;
		max_speed = 300.f;
		break;
	case M6020:
		max_current = 30000.f;
		max_speed = 200.f;
		adjspeed = 80.f;
		break;
	case M2006:
		max_current = 10000.f;
		adjspeed = 1000.f;
		max_speed = 3000.f;
		break;
	default:;
	}
}

float Motor::getDelta(float diff)
{
	if (diff <= -4096)
		diff += 8192;
	else if (diff > 4096)
		diff -= 8192;
	return diff;
}

int16_t Motor::getWord(const uint8_t high, const uint8_t low)
{
	const int16_t word = high;
	return (word << 8) + low;
}

float Motor::setRange(const float original, const float range)
{
	return std::max(std::min(range, original), -range);
}