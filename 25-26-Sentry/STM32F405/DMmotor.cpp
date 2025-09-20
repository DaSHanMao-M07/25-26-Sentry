#include "DMmotor.h"
#include "delay.h"
#include "can.h"
#include "control.h"
#include "motor.h"
#include "kalman.h"

DMMOTOR::DMMOTOR(const uint32_t ID,  CAN* hcan) //����ӿ�
	:ID(ID),  mcan(hcan)
{}

void DMMOTOR::DMmotorTransmit(uint32_t id) //�����������ݰ�
{

	if (!initial)   //��ʼֵΪfalse
	{
		MotorStart(id);//���ʹ��
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
		MotorOff(id);////���ʧ��
		motorDisablity = false;
	}
	else if (flag_zero)
	{
		MotorZero(id);//�������
		flag_zero = false;
	}
	else
	{
		if (id == 0x206)//yaw����
		{
			mcan->Transmit(id, mcan->DMmotor_temp_data_yaw, 8);
		}
		if (id == 0x109)//pitch����
		{
			mcan->Transmit(id, mcan->DMmotor_temp_data_pitch, 8);
		}
	}
}

void DMMOTOR::DMmotorOntimer(uint8_t idata[][8],uint8_t* odata)
{
	//�������ݲ����� ����������
	uint8_t id;
	if (ID == 0x06)
	{
		id = ID - 0x06;
		int direct = 0;
		int tmp_value = 0;
		tmp_value = (idata[id][1] << 8) | (idata[id][2]);//���λ��
		pos = uint_to_float(tmp_value, P_MIN, P_MAX, 16);//ת������
		tmp_value = (idata[id][3] << 4) | (idata[id][4] >> 4);//ת��
		cur_speed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//ת������
		temp_MOS = idata[id][6];//��������¶�
		temp_ROTOR = idata[id][7];//�����Ȧ�¶�
	}
	else if (ID == 0x09)//PITCH
	{
		id = ID - 0x08;
		int direct = 0;
		int tmp_value = 0;
		tmp_value = (idata[id][1] << 8) | (idata[id][2]);//���λ��
		pos = uint_to_float(tmp_value, P_MIN1, P_MAX1, 16);//ת������
		tmp_value = (idata[id][3] << 4) | (idata[id][4] >> 4);//ת��
		cur_speed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//ת������
		temp_MOS = idata[id][6];//��������¶�
		temp_ROTOR = idata[id][7];//�����Ȧ�¶�
	}
	

	//�������׼������
	if (ID == 0x06) //ID 0x06 ��yaw���� �����ٶ�ģʽ
	{
		float diff;//�ж��Ƿ���� ����������趨λ�ý��д���
		diff = set_yaw_pos - pos;//Ŀ�������ת��֮������
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
	else if (ID == 0x09) //ID 0x09 pitch���� ����λ���ٶ�ģʽ 
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

void DMMOTOR::MotorStart(uint32_t id) //���ʹ��
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
	mcan->Transmit(id, buf, 8);
}

void DMMOTOR::MotorOff(uint32_t id) //���ʧ��
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
	mcan->Transmit(id, buf, 8);
	
}

void DMMOTOR::MotorZero(uint32_t id) //�������
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
