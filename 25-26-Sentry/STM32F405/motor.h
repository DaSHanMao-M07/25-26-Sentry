#pragma once
#include <cinttypes>
#include <cstring>
#include <cmath>
#include "PID.h"
#include "kalman.h"
#include "label.h"
constexpr auto MAXSPEED = 5000;
//constexpr auto ADJUSTSPEED = 3000;

enum { ID1 = 0x201, ID2, ID3, ID4, ID5, ID6, ID7, ID8 };
enum { pre = 0, now };
enum pid_mode { speed = 0, position, autospeed, autoposition, rotation_mode };
enum motor_type { M3508, M3510, M2310, EC60, M6623, M6020, M2006 };
enum motor_mode { SPD, SPDC, POS, AUTO_Y, AUTO_P, ACE, TEST};
enum function_type { chassis, gimbal, shooter, supply };
typedef enum { UNINIT, UNCONNECTED, DISCONNECTED, FINE }motor_status_t;
#define SQRTF(x) ((x)>0?sqrtf(x):-sqrtf(-x))
#define T 1.e-3f

class Motor
{
	typedef motor_type type_t;
public:
	uint32_t ID;
	Kalman kalman{ 0.75f,67.5f };
	Kalman kalman_diff{ 0.8f,60.f };
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _autospeed, PID _autoposition, PID _rotation_mode, float _Kff_auto);//YAW
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _autospeed, PID _autoposition, float _Kff_auto);// PITCH 手操与自瞄分开调参
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position);//ACE模式
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed);
	void onTimer(uint8_t idata[][8], uint8_t* odata);

private:
	motor_status_t m_status = UNINIT;
	int32_t old_torque_current = 0;
	int32_t disconnect_count = 0;
	const int32_t disconnect_max = 10;
	void getMax(const type_t type);
	void statusIdentifier(int32_t torque_current);
	static int16_t getWord(const uint8_t high, const uint8_t low);
	static float setRange(const float original, const float range);
	type_t type;
public:
	function_type function;
	int32_t need_curcircle;
	int16_t stopAngle;
	int32_t count;
	int32_t cur_circle;
	int32_t pd;
	static float getDelta(float diff);
	uint8_t getStatus()const;
	int32_t current{}, setcurrent{}, curspeed{}, setspeed{}, curcurrent{}, temperature{};//这个current用于输出电流或者电压
	int32_t test_torque;
	int16_t adjspeed{};
	int16_t max_speed{}, max_current{};
	Kalman current_kalman{ 1.f,40.f };
	float set_angle, angle[2]{};
	float pre_set_angle;
	int32_t mode{};
	PID pid[5];
	PID static_speed{ 45.5f,0.02f,35.f };
	PID static_pos{ 2.f,0.1f,5.f };
	uint32_t IDx;
	float err;
	float powercoef;
	float Kff_autoYaw = 0.f;
	float Kff_autoPitch = 0.f;
	float Kdf_gravity = 0.f;
	float feedforward = 0.f;
	float FF_Auto(float _delta, uint32_t _ID);//自瞄前馈
	float FF_Uphill();//上坡扭矩前馈
	float FF_Friction();//阻力补偿前馈
};

//此处要根据实际不同can线上的电机数量进行更改
extern Motor can1_motor[CAN1_MOTOR_NUM];
extern Motor can2_motor[CAN2_MOTOR_NUM];
