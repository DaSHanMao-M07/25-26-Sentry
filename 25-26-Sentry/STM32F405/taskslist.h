#pragma once
#include "FreeRTOS.h"
#include "task.h"


class TASK
{
public:
	void Init();
	bool led_flag = false;
private:

};
void start_task(void* pvParameters);
void CanTransimtTask(void* pvParameters);
void LedTask(void* pvParameters);
void DecodeTask(void* pvParameters);
void MotorUpdateTask(void* pvParameters);
void ControlTask(void* pvParameters);

extern TASK task;
