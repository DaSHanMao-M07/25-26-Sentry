#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "RC.h"
#include "XUC.h"
#include "DMmotor.h"
#include "supercap.h"

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::gimbal):
			gimbal_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	mode = CONTROL::MODE::NORMAL;
	//��̨��ʼ��
	gimbal.target_yaw = gimbal.init_yaw;
	gimbal.target_pitch = gimbal.init_pitch;

	ctrl.gimbal_motor[GIMBAL::TYPE::YAW]->pre_set_angle = gimbal.init_yaw;
	ctrl.gimbal_motor[GIMBAL::TYPE::PITCH]->pre_set_angle = gimbal.init_pitch;
	DM_motorYaw.target_pos_yaw = DM_motorYaw.initial_pos_yaw;
	DM_motorPitch.target_pos_pitch = DM_motorPitch.initial_pos_pitch;
	ctrl.gimbal.angle_keep = 0.f;
	//����ǰ���������ĳ�ʼ���� 
	//һ����˵Ӧ���ڽ�������ģʽʱ��pre����setangleһ�µĳ�ʼֵ�����ձ�ȫ�̶������飬��ֱ���ڳ�ʼ����ֵ

	ctrl.STauto.robot_position[pre] = CONTROL::AUTO::POINT::recharge_point;
	ctrl.STauto.robot_position[now] = CONTROL::AUTO::POINT::recharge_point;
	//����������

}


/// ��̨
void CONTROL::GIMBAL::Update()
{

	if (ctrl.mode == CONTROL::MODE::RESET)
	{
		target_yaw = init_yaw;
		target_pitch = init_pitch;
	} 
	
	//���ô���
	if (target_yaw > 8192.0) target_yaw -= 8192.0;
	if (target_yaw < 0.0) target_yaw += 8192.0;
	if (DM_motorYaw.target_pos_yaw > PI) DM_motorYaw.target_pos_yaw -= 2 * PI;
	if (DM_motorYaw.target_pos_yaw <= -PI) DM_motorYaw.target_pos_yaw += 2 * PI;
	if (DM_motorPitch.target_pos_pitch <= DM_motorPitch.DM_pitch_min)DM_motorPitch.target_pos_pitch = DM_motorPitch.DM_pitch_min;
	if (DM_motorPitch.target_pos_pitch >= DM_motorPitch.DM_pitch_max)DM_motorPitch.target_pos_pitch = DM_motorPitch.DM_pitch_max;


	if (target_yaw >= yaw_min && target_yaw < 4096) target_yaw = yaw_min;//Сyaw��λ
	if (4096 <= target_yaw && target_yaw <= yaw_max) target_yaw = yaw_max;
	target_pitch = std::max(std::min(target_pitch, pitch_max), pitch_min);//pitch��λ

	//���
	ctrl.gimbal_motor[GIMBAL::TYPE::YAW]->set_angle = target_yaw;
	ctrl.gimbal_motor[GIMBAL::TYPE::PITCH]->set_angle = target_pitch;
	DM_motorYaw.set_yaw_pos = DM_motorYaw.target_pos_yaw;
	DM_motorPitch.set_pitch_pos = DM_motorPitch.target_pos_pitch;
	ctrl.gimbal.getEulerangle(imu_gimbalS,imu_gimbalL);
}

void CONTROL::GIMBAL::getEulerangle(IMU frameOfReferenceS,IMU frameOfReferenceL)
{
	//��ȡ��̨λ�ˣ�����������ϵ��
	

	ctrl.gimbal.S_gimbal_pitch = frameOfReferenceS.getAnglePitch();
	ctrl.gimbal.S_gimbal_yaw = frameOfReferenceS.getAngleYaw();
	ctrl.gimbal.L_gimbal_pitch = frameOfReferenceL.getAnglePitch();
	ctrl.gimbal.L_gimbal_yaw = frameOfReferenceL.getAngleYaw();

	//��¼��ʼ��
	if (ctrl.gimbal.IMUinit_flag)
	{
		float init_delta = abs(ctrl.gimbal_motor[CONTROL::GIMBAL::YAW]->angle[now] - ctrl.gimbal.init_yaw);
		float init_DMdelta = abs(DM_motorYaw.angle[now] - DM_motorYaw.initial_pos_yaw);
		if (init_delta < 0.3f && init_DMdelta < 0.02f)
		{
			S_gimbal_yaw_init = frameOfReferenceS.getAngleYaw();
			L_gimbal_yaw_init = frameOfReferenceL.getAngleYaw();
		}
		ctrl.gimbal.IMUinit_flag = false;
	}
}

void CONTROL::GIMBAL::controlGimbal(float ch_yaw, float ch_pitch) //��̨���Ƶ���Ҫ����
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//�����෴�޸���������

	target_yaw -= (float)(ch_yaw);
	target_pitch -= (float)(ch_pitch);
}

void CONTROL::GIMBAL::keepGimbal(float angleKeep, GIMBAL::TYPE type, IMU frameOfReference)//������Ŀ��� 
{
	float delta = 0;

	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.getDelta(angleKeep - frameOfReference.getAngleYaw()));
		while (fabs(delta) >= 4096.f) {
			if (delta <= -4096.f)
				delta += 8191.f;
			else if (delta >= 4096.f)
				delta -= 8192.f;
		}
		if (abs(delta) >= 3.5f)
		{
			ctrl.gimbal.target_yaw = ctrl.gimbal_motor[GIMBAL::TYPE::YAW]->angle[now] + ctrl.gimbal.imu_PID[GIMBAL::TYPE::YAW].Position(delta, 1000);
		}
	}
	else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.getDelta(angleKeep - frameOfReference.getAnglePitch()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;

		target_pitch = ctrl.gimbal_motor[PITCH]->angle[now] - imu_PID[PITCH].Position(delta, 1000.f);
	}
	else if (type == L_YAW)
	{
		delta = degreeToDMpos(ctrl.getDelta(angleKeep - ctrl.gimbal.L_gimbal_yaw));
		if (ctrl.mode == ROTATION)
		{
			if (fabs(delta) >= 0.1f)
			{
				DM_motorYaw.target_pos_yaw = DM_motorYaw.target_pos_yaw + DM_motorYaw.DM_IMU_ROTA_pid.Rota_Position(delta);
			}
		}
		else
		{
			if (fabs(delta) >= 0.01f)
			{
				DM_motorYaw.target_pos_yaw = DM_motorYaw.target_pos_yaw + DM_motorYaw.DM_IMU_pid.DM_Position(delta,6.28f);
			}
		}
	
	}
}


/// ����
void CONTROL::SHOOTER::Update()//��12����
{
	if (ctrl.mode == CONTROL::MODE::RESET)
	{
		ctrl.shooter.open_rub = false;
		ctrl.shooter.supply_bullet = false;
	}
	

	if (open_rub) 
	{
		ctrl.shooter_motor[1]->setspeed = -shoot_speed;
		ctrl.shooter_motor[2]->setspeed = shoot_speed;

	}
	else
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
		ctrl.shooter_motor[2]->setspeed = 0;
		ctrl.shooter_motor[3]->setspeed = 0;
	}

	if (supply_bullet/* && !heat_limit*/)
	{
		ctrl.supply_motor[0]->setspeed = supply_speed;
		/*ctrl.supply_motor[0]->need_curcircle = -18;
		ctrl.supply_motor[1]->need_curcircle = -18;*/

	}
	else
	{
		ctrl.supply_motor[0]->setspeed = 0;
		/*ctrl.supply_motor[0]->need_curcircle = 0;
		ctrl.supply_motor[1]->need_curcircle = 0;*/
	}
}



/// ����
void CONTROL::CHASSIS::Update()
{
	ctrl.chassis.keepDirection();

	if (ctrl.mode == RESET)
	{
		supercap.Txsuper.state = DISCHARGE;
	}
	else
	{
		if (judgement.data.robot_status_t.power_management_chassis_output)
		{
			supercap.Txsuper.state = WORKING;
		}
		else
		{
			supercap.Txsuper.state = SHUT;
		}
	}

	if (ctrl.mode == CONTROL::MODE::RESET)
	{
		speed_x = 0;
		speed_y = 0;
		speed_z = 0;
	}
	speedCalculate(speed_x , speed_y , speed_z );//ȫ���ֵ��̵���ٶȽ���
	getChassisspeed();
}

void CONTROL::CHASSIS::setChassisspeed(int32_t speedx, int32_t speedy, int32_t speedz) //�趨�����˶��ٶȣ������趨��
{
	speed_x = speedx;
	speed_y = speedy;
	speed_z = speedz;
}

void CONTROL::CHASSIS::speedCalculate(int32_t speedx, int32_t speedy, int32_t speedz) //ȫ���ֵ��̵�����ٽ��㣨�˶�ѧ����㣩
{
	float theta = 45 * PI / 180.f;//45�� �Ƕ�ת����
	speedx = 1 * speedx;
	speedy = 1 * speedy;
	//��ʱ�� 0123
	if (ctrl.STauto.finish == true)
	{	
		ctrl.chassis_motor[0]->setspeed = Ramp(-speedy * cos(theta) + speedx * cos(theta) + speedz, ctrl.chassis_motor[0]->setspeed, 40);
		ctrl.chassis_motor[1]->setspeed = Ramp(-speedy * cos(theta) - speedx * cos(theta) + speedz, ctrl.chassis_motor[1]->setspeed, 40);
		ctrl.chassis_motor[2]->setspeed = Ramp(speedy * cos(theta) - speedx * cos(theta) + speedz, ctrl.chassis_motor[2]->setspeed, 40);
		ctrl.chassis_motor[3]->setspeed = Ramp(speedy * cos(theta) + speedx * cos(theta) + speedz, ctrl.chassis_motor[3]->setspeed, 40);
	}
	else
	{
		ctrl.chassis_motor[0]->setspeed = Ramp(-speedy * cos(theta) + speedx * cos(theta) + speedz, ctrl.chassis_motor[0]->setspeed, 40);
		ctrl.chassis_motor[1]->setspeed = Ramp(-speedy * cos(theta) - speedx * cos(theta) + speedz, ctrl.chassis_motor[1]->setspeed, 40);
		ctrl.chassis_motor[2]->setspeed = Ramp(speedy * cos(theta) - speedx * cos(theta) + speedz, ctrl.chassis_motor[2]->setspeed, 40);
		ctrl.chassis_motor[3]->setspeed = Ramp(speedy * cos(theta) + speedx * cos(theta) + speedz, ctrl.chassis_motor[3]->setspeed, 40);

	}
	
}

void CONTROL::CHASSIS::getChassisspeed()
{
	float theta = 45 * PI / 180.f;
	float temp_s = 0.076;  //�ְ뾶
	float temp_r = 0.23;   //���ֵ��������ľ���

	ctrl.chassis.chassis_vx = (cos(theta) * temp_s / 4) * (ctrl.chassis_motor[0]->curspeed - ctrl.chassis_motor[1]->curspeed - ctrl.chassis_motor[2]->curspeed + ctrl.chassis_motor[3]->curspeed);
	ctrl.chassis.chassis_vy = (cos(theta) * temp_s / 4) * (-ctrl.chassis_motor[0]->curspeed - ctrl.chassis_motor[1]->curspeed + ctrl.chassis_motor[2]->curspeed + ctrl.chassis_motor[3]->curspeed);
	ctrl.chassis.chassis_w = (temp_s / (4 * temp_r)) * (ctrl.chassis_motor[0]->curspeed + ctrl.chassis_motor[1]->curspeed + ctrl.chassis_motor[2]->curspeed + ctrl.chassis_motor[3]->curspeed);

}

void CONTROL::CHASSIS::keepDirection() //����̨����Ϊ�����򣬵��̷������
{
	double s_x = speed_x, s_y = speed_y;
	double theta;

	theta = ctrl.getDelta(DMposToDegree(DM_motorYaw.pos) - DMposToDegree(DM_motorYaw.initial_pos_yaw)) / 180.f * PI; //��yawΪ������
	
	double st = sin(theta);
	double ct = cos(theta);
	speed_x = s_x * ct + s_y * st;
	speed_y = -s_x * st + s_y * ct;
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

//AUTO

void CONTROL::AUTO::stateUpdate()
{
	//�жϱ����Ƿ�ʼ
	if (judgement.data.game_status_t.game_progress == 4) { game_start = true; } 
	else { game_start = false; }

	//�ж��Ƿ�ɨ�赽����
	if (xuc.distance != -1.f&&xuc.distance!=0.f) { aim_cont++; } 
	else { aim_cont = 0; }
	if (aim_cont >= 1) { aim_lock = true; } 
	else { aim_lock = false; }

	//�ж��Ƿ���Ϊ�ܵ������˺��򱻳�ײ��Ѫ
	if (judgement.data.hurt_data_t.HP_deduction_reason == 0 || judgement.data.hurt_data_t.HP_deduction_reason == 5) { HP_deduction = true; } 
	else { HP_deduction = false; }
	
	//�жϵ����Ƿ񵽴�·����
	//������1s����δ����Ϣ����Ϊ�������˶� ��֮��0.5s�ڵ������͹���Ϣ����Ϊ�����˶�
	if (xuc.speed_x == 0 && xuc.speed_y == 0) { navi_cont++; }
	else { navi_cont = 0; }
	if (navi_cont >= 250) { navi_move = false; }
	else { navi_move = true; }

	//switch (xuc.now_point)
	//��·���㸽��50cm������Ϊ��������� ������ʮ����Ϊ�뿪������on the way��
	//if (robot_position[now] != CONTROL::AUTO::POINT::buff_point)
	//{
	//	if (fabs(xuc.point_x - recharge_point_x) <= 200.0 && fabs(xuc.point_y - recharge_point_y) <= 200.0)
	//	{
	//		robot_position[now] = CONTROL::AUTO::POINT::recharge_point;
	//	}
	//	else if (fabs(xuc.point_x - buff_point_x) <= 200.0 && fabs(xuc.point_y - buff_point_y) <= 200.0)
	//	{
	//		robot_position[now] = CONTROL::AUTO::POINT::buff_point;
	//	}
	//	else if (fabs(xuc.point_x - transit_point_x) <= 200.0 && fabs(xuc.point_y - transit_point_y) <= 200.0)
	//	{
	//		robot_position[now] = CONTROL::AUTO::POINT::transit_point;
	//	}
	//}
	
	//�������
	if (judgement.data.power_heat_data_t.shooter_17mm_1_barrel_heat > 300.f)
	{
		ctrl.shooter.heat_limit = true;
	}
	else
	{
		ctrl.shooter.heat_limit = false;
	}


	//{
	//case 0:
	//	robot_position[now] = CONTROL::AUTO::POINT::way;
	//	break;
	//case 1:
	//	robot_position[now] = CONTROL::AUTO::POINT::base_point;
	//	break;
	//case 2:
	//	robot_position[now] = CONTROL::AUTO::POINT::buff_point;
	//	break;
	//default:
	//	break;
	//}
	//robot_position[pre] = robot_position[now];
	//if (robot_position[now] == CONTROL::AUTO::POINT::way && navi_move == false) { get_stuck = true; } else { get_stuck = false; }
	//if (ctrl.mode == CONTROL::MODE::RESET)
	//{
	//	ctrl.now_mode_flag = resetmode;
	//	ctrl.pre_mode_flag = resetmode;
	// ����������
	//}
}

void CONTROL::AUTO::autoScan()
{	
	if (scan_target_yaw >= 1500.f)
	{
		scan_reversal_yaw = false;
	}
	else if (scan_target_yaw <= -1500.f)
	{
		scan_reversal_yaw = true;
	}
	if (scan_reversal_yaw)
	{
		scan_target_yaw += scan_speed_yaw;
	}
	else if (!scan_reversal_yaw)
	{
		scan_target_yaw -= scan_speed_yaw;
	}
	ctrl.gimbal.target_yaw = scan_target_yaw;

	if (scan_target_pitch >= -0.18f)
	{
		scan_reversal_pitch = false;
	}
	else if (scan_target_pitch <= -1.1f)
	{
		scan_reversal_pitch = true;
	}
	if (scan_reversal_pitch)
	{
		scan_target_pitch += scan_speed_pitch;
	}
	else if (!scan_reversal_pitch)
	{
		scan_target_pitch -= scan_speed_pitch;
	}
	DM_motorPitch.target_pos_pitch = scan_target_pitch;
}

void CONTROL::AUTO::autoFire(float _diffYAW, float _diffPITCH, IMU frameOfReference) //��λ�Ƕ���
{
	static float delta_yaw = 0, delta_pitch = 0;

	ctrl.mode = AUTOMATION;
	//ctrl.shooter.open_rub = true;

	delta_yaw = degreeToMechanical(ctrl.getDelta(_diffYAW)) - ctrl.STauto.yaw_bias;   //yaw��ƫ��
	if (delta_yaw <= -4096.f) { delta_yaw += 8192.f; }
	else if (delta_yaw >= 4096.f) { delta_yaw -= 8192.f; }
	
	if (fabs(_diffYAW) >= 0.5f)
	{
		ctrl.gimbal.target_yaw = ctrl.gimbal_motor[GIMBAL::TYPE::YAW]->angle[now] + delta_yaw;
	}
	
	delta_pitch = _diffPITCH * PI / 180.f + ctrl.STauto.pitch_bias;//pitch��ƫ��
	
	if (fabs(delta_pitch/100.f) >= 0.000001f)
	{
		DM_motorPitch.target_pos_pitch = DM_motorPitch.pos - DM_motorPitch.DM_AUTO_pid.DM_Position(delta_pitch, 3.14f);
	}
	
	

	/*if (xuc.fireadvice)

	{
		ctrl.shooter.supply_bullet = true;
	}
	else
	{
		ctrl.shooter.supply_bullet = false;
	}*/
	
}

void CONTROL::AUTO::hitFeedback()
{
	uint8_t hit_ID = judgement.data.hurt_data_t.armor_id;//װ�װ�ID��ʱ�� 0 1 2 3 ��ΪDM������λ�ó���0��װ�װ�
	uint8_t reason = judgement.data.hurt_data_t.HP_deduction_reason;//1:�ܵ��ӵ����� 5���ܵ�ײ��
	

	if (HP_deduction)
	{
		switch (hit_ID)
		{
		case 0:
			ctrl.STauto.tar_angle = ctrl.gimbal.L_gimbal_yaw_init;
			break;
		case 1:
			ctrl.STauto.tar_angle = ctrl.gimbal.L_gimbal_yaw_init + 90.f;
			break;
		case 2:
			ctrl.STauto.tar_angle = ctrl.gimbal.L_gimbal_yaw_init + 180.f;
			break;
		case 3:
			ctrl.STauto.tar_angle = ctrl.gimbal.L_gimbal_yaw_init - 90.f;
			break;
		default:
			break;
		}
	}
	ctrl.gimbal.keepGimbal(ctrl.STauto.tar_angle, CONTROL::GIMBAL::L_YAW, imu_gimbalL);
}

void CONTROL::AUTO::autoChase()
{
	float temp_speedx = xuc.speed_x / 1.2 * 3000;//ӳ���ϵ ��ͨ���ٶ��������Լ�ʵ��ã�
	float temp_speedy = xuc.speed_y;
	if (!xuc.track_flag)
	{
		ctrl.STauto.autoScan();
	}
	else
	{
		ctrl.STauto.autoFire(xuc.yaw_diff, xuc.pitch_diff, imu_gimbalS);
		ctrl.chassis.setChassisspeed(temp_speedx, temp_speedy, 0);
	}

}



void CONTROL::AUTO::autoRotation()
{
	ctrl.now_mode_flag = ROTA;
	ctrl.mode = CONTROL::MODE::ROTATION;

	if (ctrl.pre_mode_flag != ROTA)
	{
		ctrl.gimbal.angle_keep = ctrl.gimbal.L_gimbal_yaw;
		DM_motorYaw.target_pos_yaw = DM_motorYaw.pos;
	}
	//ctrl.gimbal.angle_keep = ctrl.gimbal.angle_keep + ctrl.gimbal.L_YAW_scan_speed;
	if (ctrl.gimbal.angle_keep > 180.f) ctrl.gimbal.angle_keep -= 360.f;
	if (ctrl.gimbal.angle_keep < -180.f) ctrl.gimbal.angle_keep += 360.f;

	//float temp_T = 1.f; //����Ϊ1s 
	//float speed_max = ctrl.chassis.rota_speed + 1000.f, speed_min = ctrl.chassis.rota_speed - 1500.f;
	//float rota_speed=0.f;
	//
	//rota_theta += (2 * PI) / (temp_T / 0.004);
	//rota_speed = (speed_max - speed_min) * sin(rota_theta) / 2 + (speed_max + speed_min) / 2;

	//if (aim_flag)
	//{
	//	ctrl.chassis.setChassisspeed(0, 0, ctrl.chassis.rota_speed);
	//	ctrl.gimbal.keepGimbal(ctrl.gimbal.angle_keep, CONTROL::GIMBAL::L_YAW, imu_gimbal);
	//}
	//else
	//{
	//	ctrl.gimbal.angle_keep = ctrl.gimbal.angle_keep + ctrl.gimbal.DM_speedfactor;
	//	ctrl.gimbal.keepGimbal(ctrl.gimbal.angle_keep, CONTROL::GIMBAL::L_YAW, imu_gimbal);
	//	ctrl.chassis.setChassisspeed(500, 0, ctrl.chassis.rota_speed);
	//}


	float delta = 0;
	delta = degreeToDMpos(ctrl.getDelta(ctrl.gimbal.angle_keep - ctrl.gimbal.L_gimbal_yaw));
	if (fabs(delta) >= 0.1f)
	{
		DM_motorYaw.target_pos_yaw = DM_motorYaw.target_pos_yaw + DM_motorYaw.DM_IMU_ROTA_pid.Rota_Position(delta);
	}
	//ctrl.chassis.setChassisspeed(0, 0, ctrl.chassis.rota_speed);
	ctrl.chassis.speed_z = ctrl.chassis.rota_speed;

	ctrl.pre_mode_flag = ROTA;
}


void CONTROL::AUTO::autoControl()
{
	/*if (!game_start)
	{
		return;
	}*/
	ctrl.shooter.open_rub = true;

	if (navi_move)
	{
		finish = false;
		ctrl.chassis.navi_vx = ctrl.chassis.navi_vxk * xuc.speed_x;
		ctrl.chassis.navi_vy = -1 * ctrl.chassis.navi_vyk * xuc.speed_y;
		ctrl.chassis.navi_vw = ctrl.chassis.navi_vwk * xuc.speed_w;
		ctrl.chassis.setChassisspeed(ctrl.chassis.navi_vx, ctrl.chassis.navi_vy, ctrl.chassis.navi_vw);
	}
	else
	{
		finish = true;
		if (!ctrl.STauto.aim_lock)
		{
			ctrl.mode = CONTROL::NORMAL;
			ctrl.STauto.autoScan();
			ctrl.gimbal.angle_keep = ctrl.gimbal.angle_keep + ctrl.gimbal.L_YAW_scan_speed;
		}
		else
		{
			ctrl.mode = CONTROL::AUTOMATION;
			static float delta_yaw = 0, delta_pitch = 0;
			delta_yaw = degreeToMechanical(ctrl.getDelta(xuc.yaw_diff)) - ctrl.STauto.yaw_bias;//yaw��ƫ��
			if (delta_yaw <= -4096.f) { delta_yaw += 8192.f; }
			else if (delta_yaw >= 4096.f) { delta_yaw -= 8192.f; }

			if (fabs(xuc.yaw_diff) >= 0.01f)
			{
				ctrl.gimbal.target_yaw = ctrl.gimbal_motor[GIMBAL::TYPE::YAW]->angle[now] + delta_yaw;
			}

			delta_pitch = xuc.pitch_diff * PI / 180.f + ctrl.STauto.pitch_bias;//pitch��ƫ��
			

			if (fabs(delta_pitch / 100.f) >= 0.000001f)
			{
				DM_motorPitch.target_pos_pitch = DM_motorPitch.pos - DM_motorPitch.DM_AUTO_pid.DM_Position(delta_pitch, 10.f);
			}
			
			if (xuc.fireadvice)

			{
				ctrl.shooter.supply_bullet = true;
			}
			else
			{
				ctrl.shooter.supply_bullet = false;
			}
		}

		//DM
		if (ctrl.gimbal.angle_keep > 180.f) ctrl.gimbal.angle_keep -= 360.f;
		if (ctrl.gimbal.angle_keep < -180.f) ctrl.gimbal.angle_keep += 360.f;
		float delta = 0;
		delta = degreeToDMpos(ctrl.getDelta(ctrl.gimbal.angle_keep - ctrl.gimbal.L_gimbal_yaw));
		if (fabs(delta) >= 0.1f)
		{
			DM_motorYaw.target_pos_yaw = DM_motorYaw.target_pos_yaw + DM_motorYaw.DM_IMU_ROTA_pid.Rota_Position(delta);
		}

		if (!ctrl.STauto.aim_lock)//δɨ�赽Ŀ��ʱ ��yaw��ת�� ��������������yaw�ᣨ����һ��ʼ��
		{
			ctrl.STauto.centrifugal_flag = true;
			if (judgement.data.robot_status_t.power_management_chassis_output == 0)
			{
				ctrl.chassis.speed_z = 0.f;
				ctrl.chassis.speed_x = 0.f;
			}
			else if (judgement.data.robot_status_t.power_management_chassis_output == 1)
			{
				ctrl.chassis.speed_z = ctrl.chassis.rota_speed;
				ctrl.chassis.speed_x = ctrl.chassis.centrifugal_speed;
			}
		}
		else //ɨ�赽Ŀ��ʱ ��yaw�᲻�� ��������������Լ� ʵ��ƫ��С����
		{
			ctrl.STauto.centrifugal_flag = false;
			if (judgement.data.robot_status_t.power_management_chassis_output == 0)
			{
				ctrl.chassis.speed_z = 0.f;
				ctrl.chassis.speed_x = 0.f;
			}
			else if (judgement.data.robot_status_t.power_management_chassis_output == 1)
			{
				ctrl.chassis.speed_z = ctrl.chassis.rota_speed;
				ctrl.chassis.speed_x = ctrl.chassis.centrifugal_speed;
			}
		}
	}
	
		
}	
	

/// ��������
void CONTROL::STOP()
{
	ctrl.shooter.open_rub = false;
	ctrl.shooter.supply_bullet = false;
	ctrl.chassis.setChassisspeed(0.f, 0.f, 0.f);
}


float CONTROL::getDelta(float delta)
{
	if (delta < -180.f)
	{
		delta += 360.f;
	}

	if (delta >= 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

float CONTROL::setRange(const float original, const float range)
{
	return fmaxf(fminf(range, original), -range);
}

