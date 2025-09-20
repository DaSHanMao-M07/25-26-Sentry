#include "judgement.h"
#include <cstdarg>
#include <cmath>
#include "CRC.h"

void Judgement::BuffData()
{
	if (m_uart->updateFlag)
	{
		m_uart->updateFlag = false;
		pd_Rx = xQueueReceive(*queueHandler, m_uartrx, NULL);
		m_readnum = m_uart->dataDmaNum;
		if ((m_whand + m_readnum) < (m_FIFO + BUFSIZE))
		{
			memcpy(m_whand, m_uartrx, m_readnum);
			m_whand = m_whand + m_readnum;
		}
		else if ((m_whand + m_readnum) == (m_FIFO + BUFSIZE))
		{
			memcpy(m_whand, m_uartrx, m_readnum);
			m_whand = m_FIFO;
		}
		else
		{
			const uint8_t left_size = m_FIFO + BUFSIZE - m_whand;
			memcpy(m_whand, m_uartrx, left_size);
			m_whand = m_FIFO;
			memcpy(m_whand, m_uartrx + left_size, m_readnum - left_size);
			m_whand = m_FIFO + m_readnum - left_size;
		}
		m_leftsize = m_leftsize + m_readnum;
	}

}

void Judgement::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
	huart->Init(uart_base, baud).DMARxInit();
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}


void Judgement::SendData(void)
{

	char test[3] = { "t" };
	LineDraw(&graphicData, test, UI_Graph_ADD, 4, UI_Color_Purplish_red, 6, 100, 130, 800, 500);
	//char data[6];
	//data[0] = 'B';
	//data[1] = 'U';
	//data[2] = 'L';
	//data[3] = 'L';
	//data[4] = 'E';
	//data[5] = 'T';
	/*LineDraw(&graphicData, test, UI_Graph_Line, 0, UI_Color_Purplish_red, graphicData.width, graphicData.start_x, graphicData.start_y, \
		graphicData.end_x, graphicData.end_y);*/
		//UI_ReFresh(1, &graphicData);
		//Char_ReFresh(&bulletDataStatic);
	if (testFloat > 0)
	{
		Float_Draw(&bulletDataDynamic, "b", UI_Graph_Change, 5, UI_Color_Yellow, 60, 0, 10, 500, 400, testFloat);
	}
	else
	{
		Float_Draw(&bulletDataDynamic, "b", UI_Graph_Change, 5, UI_Color_Green, 60, 0, 10, 500, 400, -testFloat);
	}

	UI_ReFresh(1, &bulletDataDynamic);
}

void Judgement::GetData(void)
{
	if (Transmit(5, m_frame) == 0)return;
	while (m_frame[0] != 0xA5)
	{
		memcpy(m_frame, m_frame + 1, 4);
		if (Transmit(1, m_frame + 4) == 0)return;
	}
	if (VerifyCRC8CheckSum(m_frame, 5))
	{
		const uint16_t datalength = static_cast<uint16_t>((m_frame[1]) | (m_frame[2] << 8));
		if (Transmit(datalength + 4, m_frame + 5) == 0)return;
		if (VerifyCRC16CheckSum(m_frame, (datalength + 9)))Decode(m_frame);
	}
}

void Judgement::Decode(uint8_t* m_frame)//未完全
{
	data.CmdID = static_cast<uint16_t>(m_frame[5] | m_frame[6] << 8);
	uint8_t* rawdata = &m_frame[7];
	switch (data.CmdID)
	{
	case 0x0001:
		data.game_status_t.game_type = static_cast<uint8_t>(rawdata[0] & 0x0F);
		data.game_status_t.game_progress = static_cast<uint8_t>(rawdata[0] >> 4);
		data.game_status_t.stage_remain_time = static_cast<uint16_t>(rawdata[1] | rawdata[2] << 8);
		break;
		//data.ext_game_status_t.SyncTimeStamp

	case 0x0002:
		data.game_result_t.winner = rawdata[0];
		break;

	case 0x0003:
		data.game_robot_HP_t.red_1_robot_HP = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.game_robot_HP_t.red_2_robot_HP = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.game_robot_HP_t.red_3_robot_HP = static_cast<uint16_t>(rawdata[4] | rawdata[5] << 8);
		data.game_robot_HP_t.red_4_robot_HP = static_cast<uint16_t>(rawdata[6] | rawdata[7] << 8);
		data.game_robot_HP_t.red_5_robot_HP = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.game_robot_HP_t.red_7_robot_HP = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.game_robot_HP_t.red_outpost_HP = static_cast<uint16_t>(rawdata[12] | rawdata[13] << 8);
		data.game_robot_HP_t.red_base_HP = static_cast<uint16_t>(rawdata[14] | rawdata[15] << 8);
		data.game_robot_HP_t.blue_1_robot_HP = static_cast<uint16_t>(rawdata[16] | rawdata[17] << 8);
		data.game_robot_HP_t.blue_2_robot_HP = static_cast<uint16_t>(rawdata[18] | rawdata[19] << 8);
		data.game_robot_HP_t.blue_3_robot_HP = static_cast<uint16_t>(rawdata[20] | rawdata[21] << 8);
		data.game_robot_HP_t.blue_4_robot_HP = static_cast<uint16_t>(rawdata[22] | rawdata[23] << 8);
		data.game_robot_HP_t.blue_5_robot_HP = static_cast<uint16_t>(rawdata[24] | rawdata[25] << 8);
		data.game_robot_HP_t.blue_7_robot_HP = static_cast<uint16_t>(rawdata[26] | rawdata[27] << 8);
		data.game_robot_HP_t.blue_outpost_HP = static_cast<uint16_t>(rawdata[28] | rawdata[29] << 8);
		data.game_robot_HP_t.blue_base_HP = static_cast<uint16_t>(rawdata[30] | rawdata[31] << 8);
		break;
	case 0x0101:
		data.event_data_t.event_data = u32_to_float(&rawdata[0]);
		break;
	case 0x0102:
		data.ext_supply_projectile_action_t.reserved = rawdata[0];
		data.ext_supply_projectile_action_t.supply_robot_id = rawdata[1];
		data.ext_supply_projectile_action_t.supply_projectile_step = rawdata[2];
		data.ext_supply_projectile_action_t.supply_projectile_num = rawdata[3];
		break;
	case 0x0104:
		data.referee_warning_t.level = rawdata[0];
		data.referee_warning_t.foul_robot_id = rawdata[1];
		data.referee_warning_t.count = rawdata[2];
		break;
	case 0x0105:
		data.dart_dart_info_t.dart_info = rawdata[0];
		data.dart_dart_info_t.dart_remaining_time = rawdata[1];
		break;
	case 0x0201:
		data.robot_status_t.robot_id = rawdata[0];
		judgementready = true;
		data.robot_status_t.robot_level = rawdata[1];
		data.robot_status_t.current_HP = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.robot_status_t.maximum_HP = static_cast<uint16_t>(rawdata[4] | rawdata[5] << 8);
		data.robot_status_t.shooter_barrel_cooling_value = static_cast<uint16_t>(rawdata[6] | rawdata[7] << 8);
		data.robot_status_t.shooter_barrel_heat_limit = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.robot_status_t.chassis_power_limit = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.robot_status_t.power_management_gimbal_output = static_cast<uint16_t>(rawdata[12] & 0x01);
		data.robot_status_t.power_management_chassis_output = static_cast<uint16_t>(rawdata[12] & 0x02) >> 1;
		data.robot_status_t.power_management_shooter_output = static_cast<uint16_t>(rawdata[12] & 0x04) >> 2;
		break;
	case 0x0202:
		powerheatready = true;
		data.power_heat_data_t.chassis_voltage = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.power_heat_data_t.chassis_current = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.power_heat_data_t.chassis_power = u32_to_float(&rawdata[4]);
		data.power_heat_data_t.chassis_power_buffer = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.power_heat_data_t.shooter_17mm_1_barrel_heat = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.power_heat_data_t.shooter_17mm_2_barrel_heat = static_cast<uint16_t>(rawdata[12] | rawdata[13] << 8);
		data.power_heat_data_t.shooter_42mm_barrel_heat = static_cast<uint16_t>(rawdata[14] | rawdata[15] << 8);
		break;
	case 0x0203:
		data.robot_pos_t.x = u32_to_float(&rawdata[0]);
		data.robot_pos_t.y = u32_to_float(&rawdata[4]);
		data.robot_pos_t.angle = u32_to_float(&rawdata[12]);
		break;
	case 0x0204:
		data.buff_t.recovery_buff = rawdata[0];
		data.buff_t.cooling_buff = rawdata[1];
		data.buff_t.defence_buff = rawdata[2];
		data.buff_t.vulnerability_buff = rawdata[3];
		data.buff_t.attack_buff = rawdata[4];
		break;
	case 0x0205:
		data.air_support_data_t.airforce_status = rawdata[0];
		data.air_support_data_t.time_remain = rawdata[1];
		break;
	case 0x0206:
		data.hurt_data_t.armor_id = static_cast<uint8_t>(rawdata[0] & 0x0F);
		data.hurt_data_t.HP_deduction_reason = static_cast<uint8_t>(rawdata[0] >> 4);
		break;
	case 0x0207:
		data.shoot_data_t.bullet_type = rawdata[0];
		data.shoot_data_t.shooter_number = rawdata[1];
		data.shoot_data_t.bullet_freq = rawdata[2];
		data.shoot_data_t.bullet_speed = u32_to_float(&rawdata[3]);
		if (prebulletspd != data.shoot_data_t.bullet_speed)
		{
			nBullet++;
			prebulletspd = data.shoot_data_t.bullet_speed;
		}
		break;
	case 0x0208:
		data.projectile_allowance_t.projectile_allowance_17mm = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.projectile_allowance_t.projectile_allowance_42mm = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.projectile_allowance_t.remaining_gold_coin = static_cast <uint16_t>(rawdata[4] | rawdata[5] << 8);
		break;

	case 0x0209:
		data.rfid_status_t.rfid_status = u32_to_float(&rawdata[0]);
		baseRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & 0x01);
		highlandRFID = static_cast<uint8_t>((data.rfid_status_t.rfid_status & 0x02) | data.rfid_status_t.rfid_status & 0x04);
		energyRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 7));
		feipoRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 8));
		outpostRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 12));
		resourseRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 16));
		break;
	case 0x020A:
		data.dart_client_cmd_t.dart_launch_opening_status = rawdata[0];
		data.dart_client_cmd_t.reserved = rawdata[1];
		data.dart_client_cmd_t.target_change_time = rawdata[2];
		data.dart_client_cmd_t.latest_launch_cmd_time = rawdata[3];
		break;
	case 0x020B:
		data.ground_robot_position_t.hero_x = u32_to_float(&rawdata[0]);
		data.ground_robot_position_t.hero_y = u32_to_float(&rawdata[4]);
		data.ground_robot_position_t.engineer_x = u32_to_float(&rawdata[8]);
		data.ground_robot_position_t.engineer_y = u32_to_float(&rawdata[12]);
		data.ground_robot_position_t.standard_3_x = u32_to_float(&rawdata[16]);
		data.ground_robot_position_t.standard_3_y = u32_to_float(&rawdata[20]);
		data.ground_robot_position_t.standard_4_x = u32_to_float(&rawdata[24]);
		data.ground_robot_position_t.standard_4_y = u32_to_float(&rawdata[28]);
		data.ground_robot_position_t.standard_5_x = u32_to_float(&rawdata[32]);
		data.ground_robot_position_t.standard_5_y = u32_to_float(&rawdata[36]);
		break;
	case 0x0303:
		data.map_command_t.target_position_x = u32_to_float(&rawdata[0]);
		data.map_command_t.target_position_y = u32_to_float(&rawdata[4]);
		data.map_command_t.cmd_keyboard = rawdata[8];
		data.map_command_t.target_robot_id = rawdata[9];
		data.map_command_t.cmd_source = rawdata[10];
	case 0x0307:
		data.map_data_t.intention = rawdata[0];
		data.map_data_t.start_position_x = static_cast<uint16_t>(rawdata[1] | rawdata[2] << 8);
		data.map_data_t.start_position_y = static_cast<uint16_t>(rawdata[3] | rawdata[4] << 8);
		memcpy(data.map_data_t.delta_x, rawdata + 5, sizeof(data.map_data_t.delta_x)); //49
		memcpy(data.map_data_t.delta_y, rawdata + 54, sizeof(data.map_data_t.delta_y));
		data.map_data_t.sender_id = static_cast<uint16_t>(rawdata[103] | rawdata[104] << 8);
		break;

	default:
		break;
	}
	
	//调试数据
	data.test_need_data.current_HP = data.robot_status_t.current_HP;
	data.test_need_data.chassis_power = data.power_heat_data_t.chassis_power;
	data.test_need_data.chassis_power_buffer = data.power_heat_data_t.chassis_power_buffer;
	data.test_need_data.bullet_freq = data.shoot_data_t.bullet_freq;
	data.test_need_data.bullet_speed = data.shoot_data_t.bullet_speed;
	data.test_need_data.shooter_17mm_1_barrel_heat = data.power_heat_data_t.shooter_17mm_2_barrel_heat;
	data.test_need_data.armor_id= data.hurt_data_t.armor_id;
	data.test_need_data.HP_deduction_reason = data.hurt_data_t.HP_deduction_reason;
}

bool Judgement::Transmit(uint32_t read_size, uint8_t* plate)
{
	if (m_leftsize < read_size)return false;

	if ((m_rhand + read_size) < (m_FIFO + BUFSIZE))
	{
		memcpy(plate, m_rhand, read_size);
		m_rhand = m_rhand + read_size;
	}
	else if ((m_rhand + read_size) == (m_FIFO + BUFSIZE))
	{
		memcpy(plate, m_rhand, read_size);
		m_rhand = m_FIFO;
	}
	else
	{
		const uint8_t left_size = m_FIFO + BUFSIZE - m_rhand;
		memcpy(plate, m_rhand, left_size);
		memcpy(plate + left_size, m_rhand = m_FIFO, read_size - left_size);
		m_rhand = m_FIFO + read_size - left_size;
	}
	m_leftsize = m_leftsize - read_size;
	return true;
}


/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    开始坐标
		End_x、End_y   结束坐标
**********************************************************************************************************/
void Judgement::LineDraw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
	uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Line;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->end_x = End_x;
	image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    开始坐标
		End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
void Judgement::Rectangle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
	uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Rectangle;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->end_x = End_x;
	image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    圆心坐标
		Graph_Radius  图形半径
**********************************************************************************************************/
void Judgement::Circle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
	uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Circle;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->radius = Graph_Radius;
}


/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_StartAngle,Graph_EndAngle    开始，终止角度
		Start_y,Start_y    圆心坐标
		x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
void Judgement::Arc_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
	uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t x_Length, uint32_t y_Length)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Arc;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->start_angle = Graph_StartAngle;
	image->end_angle = Graph_EndAngle;
	image->end_x = x_Length;
	image->end_y = y_Length;
}


/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_Size     字号
		Graph_Digit    小数位数
		Start_x、Start_x    开始坐标
		Graph_Float   要显示的变量
**********************************************************************************************************/
void Judgement::Float_Draw(float_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, \
	uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float)
{
	int i;
	for (i = 0; i < 2 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Float;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->start_angle = Graph_Size;
	image->end_angle = Graph_Digit;

	int32_t temp1 = Graph_Float * 1000;
	int32_t temp2 = temp1 / 1024;
	image->end_x = temp2;

	image->radius = temp1 - temp2 * 1024;//1->1.024*e-3

}


/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_Size     字号
		Graph_Digit    字符个数
		Start_x、Start_x    开始坐标
		*Char_Data          待发送字符串开始地址
**********************************************************************************************************/
void Judgement::Char_Draw(string_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, \
	uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char* Char_Data)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->Graph_Control.figure_name[2 - i] = imagename[i];
	image->Graph_Control.figure_tpye = UI_Graph_Char;
	image->Graph_Control.operate_tpye = Graph_Operate;
	image->Graph_Control.layer = Graph_Layer;
	image->Graph_Control.color = Graph_Color;
	image->Graph_Control.width = Graph_Width;
	image->Graph_Control.start_x = Start_x;
	image->Graph_Control.start_y = Start_y;
	image->Graph_Control.start_angle = Graph_Size;
	image->Graph_Control.end_angle = Graph_Digit;

	for (i = 0; i < Graph_Digit; i++)
	{
		image->show_Data[i] = *Char_Data;
		Char_Data++;
	}
}

/************************************************UI删除函数（使更改生效）*********************************
**参数： deleteOperator   操作数（0：空；1：删除图层；2：删除所有）
		 deleteLayer	  删除图传


Tips：
**********************************************************************************************************/
void Judgement::UIDelete(uint8_t deleteOperator, uint8_t deleteLayer)
{
	uint16_t dataLength;
	CommunatianData_graphic_t UIDeleteData;

	UIDeleteData.txFrameHeader.sof = 0xA5;
	UIDeleteData.txFrameHeader.data_length = 8;
	UIDeleteData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &UIDeleteData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	UIDeleteData.CMD = UI_CMD_Robo_Exchange;
	UIDeleteData.txID.data_cmd_id = UI_Data_ID_Del;
	UIDeleteData.txID.receiver_ID = clientId;
	UIDeleteData.txID.sender_ID = robotId;

	memcpy(m_uarttx + 5, (uint8_t*)&UIDeleteData.CMD, 8);

	m_uarttx[13] = deleteOperator;
	m_uarttx[14] = deleteLayer;

	dataLength = sizeof(CommunatianData_graphic_t) + 2;

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;
}


/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::UI_ReFresh(int cnt, graphic_data_struct_t* imageData)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 6 + cnt * 15;
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;
	switch (cnt)
	{
	case 1:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
		break;
	case 2:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
		break;
	case 5:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
		break;
	case 7:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
		break;
	default:
		break;
	}
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, imageData, cnt * sizeof(graphic_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + cnt * sizeof(graphic_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}

void Judgement::UI_ReFresh(int cnt, float_data_struct_t* floatdata)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	uint32_t graphic_data_num = sizeof(graphic_data_struct_t);
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 6 + cnt * graphic_data_num;
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;
	switch (cnt)
	{
	case 1:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
		break;
	case 2:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
		break;
	case 5:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
		break;
	case 7:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
		break;
	default:
		break;
	}
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, floatdata, cnt * graphic_data_num);
	dataLength = sizeof(CommunatianData_graphic_t) + cnt * graphic_data_num;

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}

/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::Char_ReFresh(string_data_struct_t* string_Data)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 51;
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;

	graphicData.txID.data_cmd_id = UI_Data_ID_DrawChar;
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, string_Data, sizeof(string_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + sizeof(string_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}


