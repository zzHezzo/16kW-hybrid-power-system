#include "FreeRTOS.h"
#include "semphr.h"
#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H


void freertos_demo(void);   /* ����lwIP�������� */
extern SemaphoreHandle_t gpioSemaphore;     //���յ���λ����Ϣ���ݵ��ź���
extern SemaphoreHandle_t usart3Semaphore;	//���յ�ECU��Ϣ���ݵ��ź���		
extern SemaphoreHandle_t send_PC_Semaphore;	//���͸�PC���ź���	
extern SemaphoreHandle_t send_PC_Temprature_Semaphore;	

#endif
