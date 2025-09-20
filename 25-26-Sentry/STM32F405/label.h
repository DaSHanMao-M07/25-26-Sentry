#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*
头文件中不要进行全局变量定义，可进行宏定义
全局变量用extern声明，避免头文件多次引用multiple defined
*/

#define CAN1_MOTOR_NUM 8
#define CAN2_MOTOR_NUM 4

#define CHASSIS_MOTOR_NUM 4
#define GIMBAL_MOTOR_NUM 2
#define SHOOTER_MOTOR_NUM 4
#define SUPPLY_MOTOR_NUM 2

#define RcQueueHandle Uart2QueueHandler
#define ImuQueueHandle Uart5QueueHandler
#define JudgementQueueHandle Uart3QueueHandler

#define degreeToMechanical(a) ((a)*8192.f/360.f)
#define mechanicalToDegree(a) ((a)*360.f/8192.f)
#define DMposToDegree(a) (((a)+3.1415926)/3.141593f/2*360.f)
#define degreeToDMpos(a) ((a)/360.f*2*3.141593)

#ifndef PI
#define PI 3.141593
#endif // !PI


//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
extern TaskHandle_t StartTask_Handler;
//任务函数

#define LED_TASK_PRIO		1
#define LED_STK_SIZE 		128  
extern TaskHandle_t LedTask_Handler;

#define DECODE_TASK_PRIO		3
#define DECODE_STK_SIZE 		128  
extern TaskHandle_t DecodeTask_Handler;

#define RC_TASK_PRIO		3	
#define RC_STK_SIZE 		256  
extern TaskHandle_t RcTask_Handler;

#define CONTROL_TASK_PRIO		2
#define CONTROL_STK_SIZE 		128  
extern TaskHandle_t ControlTask_Handler;

#define MOTOR_TASK_PRIO		2
#define MOTOR_STK_SIZE 		128  
extern TaskHandle_t MotorTask_Handler;

#define CANTX_TASK_PRIO		2
#define CANTX_STK_SIZE 		256 
extern TaskHandle_t CanTxTask_Handler;
