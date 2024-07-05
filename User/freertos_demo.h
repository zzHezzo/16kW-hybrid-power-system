#include "FreeRTOS.h"
#include "semphr.h"
#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H


void freertos_demo(void);   /* 创建lwIP的任务函数 */
extern SemaphoreHandle_t gpioSemaphore;     //接收到上位机消息传递的信号量
extern SemaphoreHandle_t usart3Semaphore;	//接收到ECU消息传递的信号量		
extern SemaphoreHandle_t send_PC_Semaphore;	//发送给PC的信号量	
extern SemaphoreHandle_t send_PC_Temprature_Semaphore;	

#endif
