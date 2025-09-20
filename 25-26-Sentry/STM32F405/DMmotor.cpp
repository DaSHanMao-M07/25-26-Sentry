#include "DMmotor.h"
#include "delay.h"
#include "can.h"
#include "control.h"
#include "motor.h"
#include "kalman.h"

DMMOTOR::DMMOTOR(const uint32_t ID,  CAN* hcan) //定义接口
	:ID(ID),  mcan(hcan)
{}

void DMMOTOR::DMmotorTransmit(uint32_t id) //向电机发送数据包
{

	if (!initial)   //初始值为false
	{
		MotorStart(id);//电机使能
		initial = true;
		if (flag_first)
		{
			target_pos_yaw = initial_pos_yaw;
			target_pos_pitch = initial_pos_pitch;
			flag_first = false;
		}
	}
	else if (motorDisablity)
	{
		MotorOff(id);////电机失能
		motorDisablity = false;
	}
	else if (flag_zero)
	{
		MotorZero(id);//保存零点
		flag_zero = false;
	}
	else
	{
		if (id == 0x206)//yaw轴电机
		{
			mcan->Transmit(id, mcan->DMmotor_temp_data_yaw, 8);
		}
		if (id == 0x109)//pitch轴电机
		{
			mcan->Transmit(id, mcan->DMmotor_temp_data_pitch, 8);
		}
	}
}

void DMMOTOR::DMmotorOntimer(uint8_t idata[][8],uint8_t* odata)
{
	//接收数据并解码 浮点型数据
	uint8_t id;
	if (ID == 0x06)
	{
		id = ID - 0x06;
		int direct = 0;
		int tmp_value = 0;
		tmp_value = (idata[id][1] << 8) | (idata[id][2]);//电机位置
		pos = uint_to_float(tmp_value, P_MIN, P_MAX, 16);//转浮点型
		tmp_value = (idata[id][3] << 4) | (idata[id][4] >> 4);//转速
		cur_speed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//转浮点型
		temp_MOS = idata[id][6];//电机驱动温度
		temp_ROTOR = idata[id][7];//电机线圈温度
	}
	else if (ID == 0x09)//PITCH
	{
		id = ID - 0x08;
		int direct = 0;
		int tmp_value = 0;
		tmp_value = (idata[id][1] << 8) | (idata[id][2]);//电机位置
		pos = uint_to_float(tmp_value, P_MIN1, P_MAX1, 16);//转浮点型
		tmp_value = (idata[id][3] << 4) | (idata[id][4] >> 4);//转速
		cur_speed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//转浮点型
		temp_MOS = idata[id][6];//电机驱动温度
		temp_ROTOR = idata[id][7];//电机线圈温度
	}
	

	//打包数据准备发送
	if (ID == 0x06) //ID 0x06 大yaw轴电机 采用速度模式
	{
		float diff;//判断是否过零 若过零则对设定位置进行处理
		diff = set_yaw_pos - pos;//目标角与电机转子之间的误差
		if (diff > PI) diff -= 2 * PI;
		if (diff <= -PI) diff += 2 * PI;


		//set_speed = DM_pid.Position(DM_kalman.Filter(diff), 0.5f);
		set_speed = DM_pid.Position(diff, 50.f);
		set_speed = setRange(set_speed, 50.f);


		uint8_t* vbuf;
		vbuf = (uint8_t*)&set_speed;

		odata[0] = *vbuf;
		odata[1] = *(vbuf + 1);
		odata[2] = *(vbuf + 2);
		odata[3] = *(vbuf + 3);
	}
	else if (ID == 0x09) //ID 0x09 pitch轴电机 采用位置速度模式 
	{
		uint8_t* pbuf1, * vbuf1;
		pbuf1 = (uint8_t*)&set_pitch_pos;
		vbuf1 = (uint8_t*)&set_pitch_speed;
		odata[0] = *pbuf1;
		odata[1] = *(pbuf1 + 1);
		odata[2] = *(pbuf1 + 2);
		odata[3] = *(pbuf1 + 3);
		odata[4] = *vbuf1;
		odata[5] = *(vbuf1 + 1);
		odata[6] = *(vbuf1 + 2);
		odata[7] = *(vbuf1 + 3);
	}
}

void DMMOTOR::MotorStart(uint32_t id) //电机使能
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
	mcan->Transmit(id, buf, 8);
}

void DMMOTOR::MotorOff(uint32_t id) //电机失能
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
	mcan->Transmit(id, buf, 8);
	
}

void DMMOTOR::MotorZero(uint32_t id) //保存零点
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
	mcan->Transmit(id, buf, 8);
}

void DMMOTOR::setDMmotor(float delta_pos, uint32_t id)
{
	if (id == DM_ID_Yaw)
	{
		DM_motorYaw.target_pos_yaw -= delta_pos;
	}
	
	if (id == DM_ID_Pitch)
	{
		DM_motorPitch.target_pos_pitch -= delta_pos;
	}
}
float DMMOTOR::setRange(const float original, const float range)
{
	return std::max(std::min(range, original), -range);
}
