#pragma once
#include "usart.h."
#include "FreeRTOS.h"
#include <cmath>
#include <cinttypes>

/*
×ó²¦Âës[0],ÓÒ²¦Âës[1]
ÉÏ£º1 ÖĞ£º3 ÏÂ£º2

ÓÒÒ¡¸Ë ÉÏÏÂ ch[1]
ÓÒÒ¡¸Ë ×óÓÒ ch[0]
×óÒ¡¸Ë ×óÓÒ ch[2]
×óÒ¡¸Ë ÉÏÏÂ ch[3]

*/

class RC
{
public:

	bool top_mode = false;
	bool fix = false;
	int16_t rota_direction= 1;
	float yaw_err{}, pitch_err{}, ref = 0.5f;

	struct
	{
		int16_t ch[4];
		uint8_t s[2] = { 1, 1 };
	}rc, pre_rc;

	enum POSITION { UP = 1, DOWN, MID };

	uint8_t* GetDMARx(void) { return m_frame; }

	bool judement_start = false;
	bool nuc_ctrl = false;
	void Decode();
	void onRC();
	void Update();
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);
	bool modeShifted();

private:
	QueueHandle_t* queueHandler = NULL;
	BaseType_t pd_Rx, pd_Tx;
	UART* m_uart;
	uint8_t m_frame[UART_MAX_LEN]{};
};

extern RC rc;
