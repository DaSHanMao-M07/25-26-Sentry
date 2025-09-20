#include "XUC.h"
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "math.h"
#include "RC.h"
#include "motor.h"

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	frame = m_uart->m_uartrx;
	queue_handler = &huart->UartQueueHandler;
}

void XUC::Decode()
{	
	if (frame[0] == 0xA5)
	{
		//if (!verifyCRC16CheckSum(frame, 48))
			//return;

		xuc.pitch = u8_to_float(frame + 1);
		xuc.yaw = u8_to_float(frame + 5);
		xuc.yaw_diff = u8_to_float(frame + 9);
		xuc.pitch_diff = u8_to_float(frame + 13);
		xuc.distance= u8_to_float(frame + 17);
		armor_id = (ARMOR_ID)(frame[21] & 0x0F >> 1);
		xuc.fireadvice = frame[22] & 0x01;
		xuc.velocity_x = u8_to_float(frame + 23);
		xuc.velocity_y = u8_to_float(frame + 27);
		xuc.velocity_w = u8_to_float(frame + 31);
		test_yaw = can1_motor[5].kalman.Filter(yaw);
		test_yaw_diff = can1_motor[5].kalman_diff.Filter(yaw_diff);

		if (fabs(yaw) > 300.f)
		{
			yaw = pre_yaw;
		}
		if (fabs(pitch) > 20.f)
		{
			pitch = pre_pitch;
		}
		if (fabs(yaw_diff) > 40.f)
		{
			yaw_diff = pre_yaw_diff;
		}
		if (fabs(pitch_diff) > 40.f)
		{
			pitch_diff = pre_pitch_diff;
		}
		if (fabs(velocity_x) > 999.f)
		{
			velocity_x = pre_velocity_x;
		}
		if (fabs(velocity_y) > 999.f)
		{
			velocity_y = pre_velocity_y;
		}
		if (fabs(velocity_w) > 999.f)
		{
			velocity_w = pre_velocity_w;
		}
		pre_yaw = yaw;
		pre_pitch = pitch;
		pre_yaw_diff = yaw_diff;
		pre_pitch_diff = pitch_diff;
		pre_velocity_x = pre_velocity_x;
		pre_velocity_y = pre_velocity_y;
		pre_velocity_w = pre_velocity_w;
		if (fabs(test_yaw_diff) > 50.f)
		{
			test_yaw_diff = yaw_diff;
		}
		if (fabs(test_yaw_diff) < 0.85)
		{
			fireadvice = true;
		}
		else
		{
			fireadvice = false;
		}
		
		if (distance > 0.f)
		{
			test_cont++;
		}
		else
		{
			test_cont = 0;
		}

		if (test_cont >= 50)
		{
			track_flag = true;
		}
		else
		{
			track_flag = false;
		}

	}
	
	else if (frame[0] == 0x6A)
	{
		speed_x = u8_to_double(frame + 1);
		speed_y = u8_to_double(frame + 9);
		speed_w = u8_to_double(frame + 17);
	
	}
	else if (frame[0] == 0x7A)
	{
		point_x = u8_to_double(frame + 1) * 1000; //单位mm
		point_y = u8_to_double(frame + 9) * 1000; //单位mm
		if (point_x > 15000.0 || fabs(point_x - pre_point_x) > 2000||fabs(point_x)<=1.0)
		{
			point_x = pre_point_x;
		}
		if (point_y > 10000.0 || fabs(point_y - pre_point_y) > 2000||fabs(point_y)<=1.0)
		{
			point_y = pre_point_y;
		}
		pre_point_x = point_x;
		pre_point_y = point_y;
	}
}

void XUC::Encode()
{
	count++;
	if (count % 2 == 0) 
	{
		//下面为雷达和决策接收发送数据
		tx_data[0] = 0xA6;
		
		
		if (test_navigation) 
		{
			memcpy(tx_data + 1, &(test_game_robot_HP), 36);		//所有机器人血量数据
			//memcpy(tx_data + 33, &(test_current_HP), 2);        //机器人当前血量
			//memcpy(tx_data + 35, &(test_allowance_17mm), 2);	//剩余子弹数量
			memcpy(tx_data + 37, &(test_event_data), 4);		//场地事件数据
			memcpy(tx_data + 41, &(test_game_status), 4);       //调试
			memcpy(tx_data + 45, &(test_sentry_info), 4);		//哨兵自主决策信息
		}
		else 
		{
			memcpy(tx_data + 1, &(judgement.data.game_robot_HP_t), 32);				//所有机器人血量数据
			memcpy(tx_data + 33, &(judgement.data.event_data_t), 4);				//场地事件数据
			memcpy(tx_data + 37, &(judgement.data.game_status_t), 4);				//比赛状态数据，只发比赛类型和比赛剩余时间
			memcpy(tx_data + 41, &(judgement.data.robot_status_t.current_HP), 2);   //机器人当前血量
			memcpy(tx_data + 43, &(judgement.data.projectile_allowance_t), 2);		//剩余子弹数量
			memcpy(tx_data + 45, &(judgement.data.sentry_info_t), 4);				//哨兵自主决策信息
		}
			
		
		
		appendCRC16CheckSum(tx_data, 51);						//添加校验位
		m_uart->UARTTransmit(tx_data, 51);
	}
	else
	{
		own_color = judgement.data.robot_status_t.robot_id <= 7 ? RED : BLUE;
		TxNuc.header = 0x5A;
		TxNuc.detect_color = !own_color; //id大于七为蓝方，则瞄准红装甲板
		TxNuc.reset_tracker = 0;
		TxNuc.reserved = 15;
		TxNuc.roll = imu_gimbalS.getAngleRoll();
		TxNuc.pitch = imu_gimbalS.getAnglePitch() - imu_gimbalS.init_pitch;
		TxNuc.yaw = imu_gimbalS.getAngleYaw();
		TxNuc.aim_x = aim_x;
		TxNuc.aim_y = aim_y;
		TxNuc.aim_z = aim_z;
		TxNuc.checksum = 0;


		memcpy(tx_data, &TxNuc, 2);
		memcpy(tx_data + 2, &(TxNuc.roll), 24);
		appendCRC16CheckSum(tx_data, 28);
		m_uart->UARTTransmit(tx_data, 28);
	}
}

uint16_t XUC::getCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t ch_data;

	if (pchMessage == nullptr) return 0xFFFF;
	while (dwLength--) 
	{
		ch_data = *pchMessage++;
		(wCRC) =
			((uint16_t)(wCRC) >> 8) ^ CRC_TAB[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
	}

	return wCRC;
}

uint32_t XUC::verifyCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_expected = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return false;

	w_expected = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);
	return (
		(w_expected & 0xff) == pchMessage[dwLength - 2] &&
		((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void XUC::appendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_crc = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return;

	w_crc = getCRC16CheckSum(reinterpret_cast<uint8_t*>(pchMessage), dwLength - 2, CRC16_INIT);

	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}