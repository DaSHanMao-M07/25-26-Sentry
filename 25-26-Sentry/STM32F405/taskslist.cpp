#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "judgement.h"
#include "XUC.h"
#include "DMmotor.h"
#include "supercap.h"
#include "power.h"

void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

	xTaskCreate((TaskFunction_t)LedTask,
		(const char*)"LedTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();
		imu_gimbalS.Decode();
		imu_gimbalL.Decode();
		xuc.Decode();
		judgement.GetData();
		//supercap.decode();
		
		vTaskDelay(4);
	}
}

void MotorUpdateTask(void* pvParameters)
{
	while (true)
	{
		for (auto& motor : can1_motor)motor.onTimer(can1.data, can1.temp_data);
		for (auto& motor : can2_motor)motor.onTimer(can2.data, can2.temp_data);
		DM_motorYaw.DMmotorOntimer(can2.DMmotor_data, can2.DMmotor_temp_data_yaw);
		DM_motorPitch.DMmotorOntimer(can1.DMmotor_data, can1.DMmotor_temp_data_pitch);
		vTaskDelay(4);
	}
}

void CanTransimtTask(void* pvParameters)
{
	while (true)
	{
		//judgement.BuffData();
		switch (timer.counter % 5)
		{
		case 0:
			can1.Transmit(0x200, can1.temp_data);
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			break;
		case 2:
			can2.Transmit(0x200, can2.temp_data);
			break;
		case 3:
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		case 4:
			DM_motorYaw.DMmotorTransmit(0x206);
			DM_motorPitch.DMmotorTransmit(0x109);
			if (timer.counter % 250 == 4)
			{
				DM_motorYaw.MotorStart(0x206);
				DM_motorPitch.MotorStart(0x109);
			}
			break;
		}
		vTaskDelay(4);
	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
		ctrl.STauto.stateUpdate();
		ctrl.chassis.Update();
		ctrl.gimbal.Update();
		ctrl.shooter.Update();
		rc.Update();
		xuc.Encode();
		/*if ((timer.counter - timer.precounter) > 100.f)
		{
			supercap.encode();
			timer.precounter = timer.counter;
		}
		power.RCS_PowerUpdate();*/
		judgement.BuffData();
		vTaskDelay(4);
	}
}

void LedTask(void* pvParameters)
{
	while (true)
	{
		vTaskDelay(500);
	}
}





