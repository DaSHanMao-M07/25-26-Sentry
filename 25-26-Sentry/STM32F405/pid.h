#pragma once
#include "stm32f4xx_hal.h"
#include <algorithm>

enum { INTEGRATE = 0, LLAST = 0, LAST = 1, NOW = 2 };
#define FILTER 2
class PID
{
public:
	PID(void)
		: m_Kp(0.f)
		, m_Ti(0.f)
		, m_Td(0.f), m_alpha(0.f) {}
	PID(float Kp, float Ti, float Td, float alpha = 0.0f)
		: m_Kp(Kp)
		, m_Ti(Ti)
		, m_Td(Td), m_alpha(alpha) {}
	PID(float Kp, float Ti, float Td, float i_min, float i_max)
		: m_Kp(Kp)
		, m_Ti(Ti)
		, m_Td(Td)
		, min_limit(i_min), max_limit(i_max) {}


	float Filter(float delta)      //滤波，取平均值
	{
		float sum = 0;
		m_filter[m_filterindex++] = delta;
		if (m_filterindex == FILTER)m_filterindex = 0;
		for (int16_t t = 0; t != FILTER; t++)
			sum += m_filter[t];
		return sum / static_cast<float>(FILTER);
	}
	float Delta(float error)             //增量式PID
	{
		m_error[LLAST] = m_error[LAST] * 0.92f;
		m_error[LAST] = m_error[NOW] * 0.92f;
		m_error[NOW] = error * 1.08f;

		return m_Kp * (m_error[NOW] - m_error[LAST]) + m_Ti * m_error[NOW] + m_Td * (m_error[NOW] - 2 * m_error[LAST] + m_error[LLAST]);
	}
	float Position(float error, float max_limit)
	{
		m_error[NOW] = error;
		m_error[INTEGRATE] += m_error[NOW];
		m_error[INTEGRATE] = std::max(std::min(m_error[INTEGRATE], max_limit), -max_limit);
		//不完全微分
		this->m_lderivative = m_Td * (1.f - m_alpha) * (m_error[NOW] - m_error[LAST]) + m_alpha * m_lderivative;
		const float result = this->m_error[NOW] * this->m_Kp + this->m_error[INTEGRATE] * this->m_Ti + this->m_lderivative;
		m_error[LAST] = m_error[NOW];
		output = result;
		return result;
	}
	
	float DM_Position(float error, float max_limit)//范围不一样 系数比较小 单独拎出来
	{
		m_error[NOW] = error;
		m_error[INTEGRATE] += m_error[NOW];
		m_error[INTEGRATE] = std::max(std::min(m_error[INTEGRATE], max_limit), -max_limit);
		//不完全微分
		this->m_lderivative = m_Td * (1.f - m_alpha) * (m_error[NOW] - m_error[LAST]) + m_alpha * m_lderivative;
		const float result = this->m_error[NOW] * this->m_Kp*0.001 + this->m_error[INTEGRATE] * this->m_Ti*0.001 + this->m_lderivative;
		m_error[LAST] = m_error[NOW];
		output = result;
		return result;
	}
	float Rota_Position(float error)
	{
		m_error[NOW] = error;
		if(m_error[NOW] < 0)
			m_error[INTEGRATE] += m_error[NOW] * n_compensate;
		if(m_error[NOW] > 0)
			m_error[INTEGRATE] += m_error[NOW] * p_compensate;
		m_error[INTEGRATE] = std::max(std::min(m_error[INTEGRATE], max_limit), -max_limit);
		if (m_error[INTEGRATE] > -min_limit) {
			m_error[INTEGRATE] = -min_limit;
		}
		//不完全微分
		this->m_lderivative = m_Td * (1.f - m_alpha) * (m_error[NOW] - m_error[LAST]) + m_alpha * m_lderivative;
		const float result = this->m_error[NOW] * this->m_Kp*0.001 + this->m_error[INTEGRATE] * this->m_Ti*0.001 + this->m_lderivative;
		m_error[LAST] = m_error[NOW];
		output = result;
		return result;
	}
	float m_Kp, m_Ti, m_Td;
	float max_limit = 1000.0f;
	float min_limit = 0.f;
	float m_error[3] = { 0 };
	float output;
	float p_compensate = 1.f, n_compensate = 1.f;
private:
	float m_alpha = 0.f, m_lderivative = 0.f;
	float m_filter[FILTER] = { 0 };
	uint16_t m_filterindex = 0;
};
