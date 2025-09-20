#include "label.h"
#include "RC.h"
#include "control.h"
#include "XUC.h"


void RC::onRC()
{
	if (rc.s[0] == UP && rc.s[1] == UP) //Reset模式
	{
		if (modeShifted())
		{
			ctrl.STOP();
		}
		ctrl.mode = CONTROL::MODE::RESET;
		
		DM_motorYaw.target_pos_yaw = DM_motorYaw.initial_pos_yaw;
		DM_motorPitch.target_pos_pitch = DM_motorPitch.initial_pos_pitch;
		if (rc.ch[0] != 0 || rc.ch[1] != 0 || rc.ch[2] != 0 || rc.ch[3] != 0)
		{
			DM_motorYaw.motorDisablity = true;
			DM_motorPitch.motorDisablity = true;
		}
		ctrl.STauto.finish = false;

	}
	else if (rc.s[0] == UP && rc.s[1] == MID) //调试底盘
	{
		if (modeShifted())
		{
			ctrl.STOP();
		}
		
		ctrl.mode = CONTROL::NORMAL;
		ctrl.chassis.setChassisspeed(rc.ch[3] / 660.f * ctrl.chassis.max_speed, rc.ch[2] / 660.f * ctrl.chassis.max_speed, rc.ch[0] / 660.f * ctrl.chassis.rota_speed);
	}
	else if (rc.s[0] == UP && rc.s[1] == DOWN) //调试云台
	{
	
		if (modeShifted())
		{
			DM_motorYaw.initial = false;
			ctrl.STOP();
		}
		ctrl.mode = CONTROL::NORMAL;

		
		DM_motorYaw.setDMmotor(rc.ch[2] / 66000.f * 5 * ctrl.gimbal.DM_speedfactor, DM_motorYaw.DM_ID_Yaw);
		DM_motorPitch.setDMmotor(rc.ch[3] / 66000.f * 5 * ctrl.gimbal.DM_speedfactor, DM_motorPitch.DM_ID_Pitch);
		ctrl.gimbal.controlGimbal(rc.ch[0] / 660.f * ctrl.gimbal.manualYawspeed, 0);
		
	}

	//else if (rc.s[0] == MID && rc.s[1] == UP) //扫描
	//{
	//	if (modeShifted())
	//	{
	//		ctrl.STOP();
	//	}
	//	ctrl.mode = CONTROL::AUTOMATION;
	//	//ctrl.shooter.open_rub = true;
	//	ctrl.STauto.autoFire(xuc.yaw_diff, xuc.pitch_diff, imu_gimbalS);
	//	//DM_motorPitch.setDMmotor(rc.ch[3] / 66000.f * ctrl.gimbal.DM_speedfactor, DM_motorPitch.DM_ID_Pitch);
	//	/*if (xuc.fireadvice)
	//	{
	//		if (rc.ch[1] > 440.f)
	//		{
	//			ctrl.shooter.supply_bullet = true;
	//		}
	//		else if (rc.ch[1] > -330.f && rc.ch[1] < 110.f)
	//		{
	//			ctrl.shooter.supply_bullet = false;
	//		}
	//		else if (rc.ch[1] < -330.f)
	//		{
	//			ctrl.shooter.open_rub = false;
	//		}
	//	}
	//	else
	//	{
	//		ctrl.shooter.supply_bullet = false;
	//	}*/
	//	
	//	
	//}

	//else if (rc.s[0] == MID && rc.s[1] == MID) //调试导航 0-200 200-400 400以上手动
	//{
	//	if (modeShifted())
	//	{
	//		ctrl.STOP();
	//		ctrl.gimbal.angle_keep = ctrl.gimbal.L_gimbal_yaw;
	//	}
	//
	///*	xuc.test_navigation = true;
	//	ctrl.chassis.navi_vx = ctrl.chassis.navi_vxk * xuc.speed_x;
	//	ctrl.chassis.navi_vy = -1 * ctrl.chassis.navi_vyk * xuc.speed_y;
	//	ctrl.chassis.navi_vw = ctrl.chassis.navi_vwk * xuc.speed_w;
	//	ctrl.chassis.setChassisspeed(ctrl.chassis.navi_vx, ctrl.chassis.navi_vy, ctrl.chassis.navi_vw);
	//	if (rc.ch[0] > 330.f)
	//	{
	//		xuc.test_game_robot_HP.test_current_HP = 300.f;
	//	}
	//	else if (rc.ch[0] < -330.f)
	//	{
	//		xuc.test_game_robot_HP.test_current_HP = 50.f;
	//	}
	//	if (rc.ch[1] >= 550.f && rc.ch[3] >= 550.f)
	//	{
	//		xuc.test_game_robot_HP.test_current_HP = 500.f;
	//	}*/
	//	ctrl.mode = CONTROL::ROTATION;
	//	ctrl.gimbal.angle_keep += rc.ch[2] / 110.f * ctrl.gimbal.DM_speedfactor;
	//	if (ctrl.gimbal.angle_keep > 180.f) ctrl.gimbal.angle_keep -= 360.f;
	//	if (ctrl.gimbal.angle_keep < -180.f) ctrl.gimbal.angle_keep += 360.f;
	//	ctrl.gimbal.keepGimbal(ctrl.gimbal.angle_keep, CONTROL::GIMBAL::L_YAW, imu_gimbalL);
	//	ctrl.chassis.setChassisspeed(rc.ch[3] / 660.f * ctrl.chassis.max_speed, 0, ctrl.chassis.rota_speed);
	//	
	//}

	//else if (rc.s[0] == MID && rc.s[1] == DOWN) //小陀螺
	//{
	//	if (modeShifted())
	//	{
	//		ctrl.STOP();
	//		ctrl.gimbal.angle_keep = ctrl.gimbal.L_gimbal_yaw;
	//	}
	//	ctrl.mode = CONTROL::ROTATION;
	//	ctrl.gimbal.angle_keep += rc.ch[2] / 110.f * ctrl.gimbal.DM_speedfactor;
	//	if (ctrl.gimbal.angle_keep > 180.f) ctrl.gimbal.angle_keep -= 360.f;
	//	if (ctrl.gimbal.angle_keep < -180.f) ctrl.gimbal.angle_keep += 360.f;
	//	ctrl.gimbal.keepGimbal(ctrl.gimbal.angle_keep, CONTROL::GIMBAL::L_YAW, imu_gimbalL);
	//	ctrl.chassis.setChassisspeed(rc.ch[3] / 660.f * ctrl.chassis.max_speed, 0, rc.ch[0] / 660.f * ctrl.chassis.rota_speed);

	//}

	//else if (rc.s[0] == DOWN && rc.s[1] == UP) //调试发射
	//{

	//	if (modeShifted())
	//	{
	//		ctrl.chassis.setChassisspeed(0.f, 0.f , 0.f);
	//	}
	//	ctrl.mode = CONTROL::NORMAL;
	//	ctrl.shooter.open_rub = true;
	//	ctrl.gimbal.controlGimbal(rc.ch[2] / 3000.f * ctrl.gimbal.manualYawspeed, 0);
	//	DM_motorPitch.setDMmotor(rc.ch[3] / 66000.f * 5 * ctrl.gimbal.DM_speedfactor, DM_motorPitch.DM_ID_Pitch);

	//	if (rc.ch[1] > 440.f)
	//	{
	//		ctrl.shooter.supply_bullet = true;
	//	}
	//	else if (rc.ch[1] > -330.f && rc.ch[1] < 110.f)
	//	{
	//		ctrl.shooter.supply_bullet = false;
	//	}
	//	else if (rc.ch[1] < -330.f)
	//	{
	//		ctrl.shooter.open_rub = false;
	//	}
	//}

	//else if (rc.s[0] == DOWN && rc.s[1] == MID) // 不走中心
	//{
	//	if (modeShifted())
	//	{

	//	}
	//	ctrl.STauto.autoFire(xuc.yaw_diff, xuc.pitch_diff, imu_gimbalS);
	//	

	//}
	//else if (rc.s[0] == DOWN && rc.s[1] == DOWN) //测试扫描 测试决策发点
	//{
	//
	//	if (modeShifted())
	//	{
	//		ctrl.mode = CONTROL::NORMAL;
	//	}

	//	/*xuc.test_navigation = true;
	//	ctrl.STauto.autoScan();
	//	
	//	if (rc.ch[0] > 330.f)
	//	{
	//		xuc.test_game_robot_HP.test_current_HP = 300.f;
	//	}
	//	else if (rc.ch[0] < -330.f)
	//	{
	//		xuc.test_game_robot_HP.test_current_HP = 50.f;
	//	}
	//	if (rc.ch[1] >= 550.f && rc.ch[3] >= 550.f)
	//	{
	//		xuc.test_game_robot_HP.test_current_HP = 500.f;
	//	}*/

	//	ctrl.STauto.autoControl();
	//}
	
	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

}

void RC::Decode()
{
	pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);
}

void RC::Update()
{
	onRC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

bool RC::modeShifted()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}
