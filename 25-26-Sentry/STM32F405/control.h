#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"
#include "DMmotor.h"

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

class CONTROL final
{
public:
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* gimbal_motor[GIMBAL_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};

	bool outpost_exsist;
	bool invicible = true;

	enum MODE { RESET = 0, NORMAL, FOLLOW, ROTATION, AUTOMATION,TEST } mode;
	enum { pre = 0, now };
	enum { resetmode = 0, ROTA };
	int pre_mode_flag = 0;
	int now_mode_flag = 0;

	struct CHASSIS
	{
		uint16_t max_speed = 3500, rota_speed = 4250.f, centrifugal_speed = 500.f;
		//6000 场地内小陀螺起步扣一点缓冲功率 够顶 4750
	
		int32_t speed_x{}, speed_y{}, speed_z{};
		float chassis_vx, chassis_vy, chassis_w;
		float navi_vx = 0.f, navi_vy = 0.f, navi_vw; //储存导航速度
		float navi_vxk = 1500.f, navi_vyk = 1500.f, navi_vwk = 500.f;//调节导航速度系数

		void Update();
		void setChassisspeed(int32_t speedx, int32_t speedy, int32_t speedz); //设定宏观下车运动速度（解算前）
		void speedCalculate(int32_t speedx, int32_t speedy, int32_t speedz); //轮速解算
		void getChassisspeed();
		void keepDirection();//以云台为正方向前进
		
		float Ramp(float setval, float curval, uint32_t RampSlope);

		
	private:
	
	};

	struct GIMBAL
	{
		enum TYPE { YAW = 0, PITCH, L_YAW };
		bool IMUinit_flag = true;
		float target_pitch = 6500, target_yaw = 0.f;
		float init_pitch = 3075.f, init_yaw = 0.f, pitch_min = 2500.f, pitch_max = 3700.f, yaw_max = 2000.f, yaw_min = 6192.f;//初始角 限位角
		float DM_speedfactor = 0.15f, manualYawspeed = 20.f, manualPitchspeed = 10.f; //遥控手动云台速度
		float angle_keep = 0.f;//用于控制大yaw
		float L_YAW_scan_speed = 0.3f,pitch_scan_speed;//大yaw pitch扫描速率
		float S_gimbal_yaw, S_gimbal_pitch, L_gimbal_yaw, L_gimbal_pitch;//大、小云台姿态
		float S_gimbal_yaw_init,L_gimbal_yaw_init;
		PID imu_PID[3] = { {2.f,0.0f,0.1f}, {0.35f,0.f,0.f, 0.f, 1000.f},{22.50f,0.0f,0.0f, 10.f, 10.f} };//陀螺仪控制云台时的pid

		void keepGimbal(float angleKeep, GIMBAL::TYPE type, IMU frameOfReference);
		void Update();
		void controlGimbal(float ch_yaw, float ch_pitch);
		void getEulerangle(IMU frameOfReferenceS, IMU frameOfReferenceL);//获取大、小云台姿态 包括小云台pitch、yaw 大云台yaw、倾角
	};

	struct SHOOTER
	{

		bool open_rub = false, supply_bullet = false;
		bool fullheat_shoot = false;
		bool heat_limit = false;
		int16_t shoot_speed = 6200;
		int16_t supply_speed = -3000;

		void Update();

	private:
		int32_t shooter_count, last_heat;
	};

	struct AUTO
	{
		void autoScan();
		void autoFire(float _angleKeepYAW, float _angleKeepPITCH, IMU frameOfReference);
		void hitFeedback();
		void autoChase();
		void autoRotation();
		void autoControl();
		void stateUpdate();
		void autoNavigation();

	
		int navi_cont = 0;
		int aim_cont = 0;

		float tar_angle = 0.f; //受击反馈模式下用于控制大yaw
		
		double goal_point_x, goal_point_y;
		double buff_point_x = 1000.0, buff_point_y = 500.0;//增益点
		double transit_point_x = 1000.0, transit_point_y = 10.0;//中间点
		double recharge_point_x = 10.0, recharge_point_y = 10.0;//补给点

		double delta_x = 0.0, delta_y = 0.0;
		bool reach_point_x = false;
		bool reach_point_y = false;
		bool centrifugal_flag = false;
		bool finish = false;
		bool aim_lock = false;

		enum POINT { way = 0, recharge_point, transit_point, buff_point };
		POINT robot_position[2];
	private:
		float scan_speed_yaw = 7.5f, scan_target_yaw;
		float scan_speed_pitch = 0.007f, scan_target_pitch=-0.5;
		bool scan_reversal_yaw = false, scan_reversal_pitch = false;
		float rota_theta = 0.f;
		float yaw_bias = 0.f, pitch_bias = 0.f;//1.5 1.75

		bool game_start = false;
		bool HP_deduction = false;
		bool get_stuck = false;
		bool navi_move = false;
		
		
	};

	CHASSIS chassis;
	GIMBAL gimbal;
	SHOOTER shooter;
	AUTO STauto;

	float Plimit = 1.f;
	
	bool test_flag = false;
	

	static float setRange(const float original, const float range);
	float getDelta(float delta);
	void STOP();
	void Init(std::vector<Motor*> motor);


private:

};

extern CONTROL ctrl;