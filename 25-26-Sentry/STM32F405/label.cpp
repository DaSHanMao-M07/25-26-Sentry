#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "label.h"

/*
����������
*/
TaskHandle_t StartTask_Handler;
TaskHandle_t LedTask_Handler;
TaskHandle_t DecodeTask_Handler;
TaskHandle_t ControlTask_Handler;
TaskHandle_t MotorTask_Handler;
TaskHandle_t CanTxTask_Handler;

