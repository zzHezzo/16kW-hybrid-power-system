#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "lwip_comm.h"
#include "lwipopts.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tcp_server_demo.h" 
#include "socket_tcp.h"
#include "zzh.h"
#include "timer.h"
#include "freertos_demo.h"
//#include "queue.h"
//#include "semphr.h"
#include "spi.h"



/******************************************************************************************************/
/*�����ź�������*/
//static QueueHandle_t xQueue2;
//extern volatile uint16_t valve_servo , battery_status , Electronic_Speed_Controller_relay, rectifier_relay;
//extern volatile uint16_t first_cylinder_temprature, second_cylinder_temprature,
//	engine_working_state, engine_speed, engine_warm_up, engine_idling_set,
//	/*		10					         12				     18				          24	*/
//	air_inflow, throttle_position;
//	/*	28				     74		*/

SemaphoreHandle_t gpioSemaphore;     //���յ���λ����Ϣ���ݵ��ź���
SemaphoreHandle_t usart3Semaphore;	//���յ�ECU��Ϣ���ݵ��ź���		
SemaphoreHandle_t send_PC_Semaphore;	//���͸�PC���ź���	
SemaphoreHandle_t send_PC_Temprature_Semaphore;	//���͸�PC���¶��ź���	
/******************************************************************************************************/



/*FreeRTOS����*/
/* START_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define START_TASK_PRIO         5           /* �������ȼ� */
#define START_STK_SIZE          128         /* �����ջ��С */
TaskHandle_t StartTask_Handler;             /* ������ */
void start_task(void *pvParameters);        /* ������ */

/* LWIP_DEMO ���� ���� ����λ��ͨѶ������������
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define LWIP_DMEO_TASK_PRIO     11          /* �������ȼ� */
#define LWIP_DMEO_STK_SIZE      128        /* �����ջ��С */
TaskHandle_t LWIP_Task_Handler;             /* ������ */
void lwip_demo_task(void *pvParameters);    /* ������ */

/* LED_TASK ���� ���� 
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define LED_TASK_PRIO           10          /* �������ȼ� */
#define LED_STK_SIZE            128         /* �����ջ��С */
TaskHandle_t LEDTask_Handler;               /* ������ */
void led_task(void *pvParameters);          /* ������ */

///* MsgPro_TASK ���� ����
// * ����: ������ �������ȼ� ��ջ��С ��������
// */
//#define MsgPro_TASK_PRIO           3          /* �������ȼ� */
//#define MsgPro_STK_SIZE            128         /* �����ջ��С */
//TaskHandle_t MsgProTask_Handler;               /* ������ */
//static void vTaskMsgPro(void *pvParameters);          /* ������ */

/* Controller_relay_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define Controller_relay_TASK_PRIO        7          /* �������ȼ� */
#define Controller_relay_SIZE            128         /* �����ջ��С */
TaskHandle_t Controller_relay_TASK_Handler;               /* ������ */
void controller_relay_task(void *pvParameters);          /* ������ */

/* RS232_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define RS232_TASK_PRIO          8          /* �������ȼ� */
#define RS232_STK_SIZE          128         /* �����ջ��С */
TaskHandle_t RS232Task_Handler;             /* ������ */
void rs232_TASK(void *pvParameters);        /* ������ */

/* Temprature_TASK ���� ����
 * ����: ������ �������ȼ� ��ջ��С ��������
 */
#define Temprature_TASK_PRIO           9           /* �������ȼ� */
#define Temprature_STK_SIZE            256         /* �����ջ��С */
TaskHandle_t TempratureTask_Handler;               /* ������ */
void temprature_task(void *pvParameters);          /* ������ */
/******************************************************************************************************/


/**
 * @breif       ����UI
 * @param       mode :  bit0:0,������;1,����ǰ�벿��UI
 *                      bit1:0,������;1,���غ�벿��UI
 * @retval      ��
 */
void lwip_test_ui(uint8_t mode)
{
    uint8_t speed;
    uint8_t buf[30];
    
    if (mode & 1<< 0)
    {
        lcd_show_string(6, 10, 200, 32, 32, "STM32", DARKBLUE);
        lcd_show_string(6, 40, 200, 24, 24, "lwIP connect PC", DARKBLUE);
        lcd_show_string(6, 70, 200, 16, 16, "ZZH@XMU", DARKBLUE);
    }
    
    if (mode & 1 << 1)
    {
        lcd_show_string(5, 110, 200, 16, 16, "lwIP Init Successed", MAGENTA);
        
        if (g_lwipdev.dhcpstatus == 2)
        {
            sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",g_lwipdev.ip[0],g_lwipdev.ip[1],g_lwipdev.ip[2],g_lwipdev.ip[3]);     /* ��ʾ��̬IP��ַ */
        }
        else
        {
            sprintf((char*)buf,"Static IP:%d.%d.%d.%d",g_lwipdev.ip[0],g_lwipdev.ip[1],g_lwipdev.ip[2],g_lwipdev.ip[3]);    /* ��ӡ��̬IP��ַ */
        }
        
        lcd_show_string(5, 130, 200, 16, 16, (char*)buf, MAGENTA);
        
        speed = ethernet_chip_get_speed();      /* �õ����� */
        
        if (speed)
        {
            lcd_show_string(5, 150, 200, 16, 16, "Ethernet Speed:100M", MAGENTA);
        }
        else
        {
            lcd_show_string(5, 150, 200, 16, 16, "Ethernet Speed:10M", MAGENTA);
        }
    }
}

/**
 * @breif       freertos_demo
 * @param       ��
 * @retval      ��
 */
void freertos_demo(void)
{
    /* start_task���� */
    xTaskCreate((TaskFunction_t )start_task,
                (const char *   )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void *         )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t * )&StartTask_Handler);

    vTaskStartScheduler(); /* ����������� */
}

/**
 * @brief       start_task
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void start_task(void *pvParameters)
{
    pvParameters = pvParameters;
    
    g_lwipdev.lwip_display_fn = lwip_test_ui;
    
    g_lwipdev.lwip_display_fn = lwip_test_ui;
    
    lwip_test_ui(1);    /* ���غ�ǰ����UI */
    
    while (lwip_comm_init() != 0)
    {
        lcd_show_string(30, 110, 200, 16, 16, "lwIP Init failed!!", RED);
        delay_ms(500);
        lcd_fill(30, 50, 200 + 30, 50 + 16, WHITE);
        lcd_show_string(30, 110, 200, 16, 16, "Retrying...       ", RED);
        delay_ms(500);
        LED1_TOGGLE();
    }
    
    while (!ethernet_read_phy(PHY_SR))  /* ���MCU��PHYоƬ�Ƿ�ͨ�ųɹ� */
    {
        printf("MCU��PHYоƬͨ��ʧ�ܣ������·����Դ�룡������\r\n");
    }
    
    while ((g_lwipdev.dhcpstatus != 2)&&(g_lwipdev.dhcpstatus != 0XFF))  /* �ȴ�DHCP��ȡ�ɹ�/��ʱ��� */
    {
        vTaskDelay(5);
    }
		
    /*������ֵ�ź���*/
		usart3Semaphore = xSemaphoreCreateBinary();
		gpioSemaphore = xSemaphoreCreateBinary();
		send_PC_Semaphore = xSemaphoreCreateBinary();
		send_PC_Temprature_Semaphore = xSemaphoreCreateBinary();
    
    taskENTER_CRITICAL();           /* �����ٽ��� */
		tcpecho_init();/*��Ϊ����������*/
    /* ����lwIP���� */
    xTaskCreate((TaskFunction_t )lwip_demo_task,
                (const char*    )"lwip_demo_task",
                (uint16_t       )LWIP_DMEO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LWIP_DMEO_TASK_PRIO,
                (TaskHandle_t*  )&LWIP_Task_Handler);

    /* LED�������� */
    xTaskCreate((TaskFunction_t )led_task,
                (const char*    )"led_task",
                (uint16_t       )LED_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )LED_TASK_PRIO,
                (TaskHandle_t*  )&LEDTask_Handler);
    
//		/* MsgPro_TASK���� */
//    xTaskCreate((TaskFunction_t )vTaskMsgPro,
//                (const char*    )"vTaskMsgPro",
//                (uint16_t       )MsgPro_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )MsgPro_TASK_PRIO,
//                (TaskHandle_t*  )&MsgProTask_Handler);
								
		/* �̵�������ָ������� */
    xTaskCreate((TaskFunction_t )controller_relay_task,
                (const char*    )"controller_relay_task",
                (uint16_t       )Controller_relay_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Controller_relay_TASK_PRIO,
                (TaskHandle_t*  )&Controller_relay_TASK_Handler);
								
		/* RS232���Ͳ�ѯECUָ��͸���ECU���ջ�����ָ��ά��ȫ�ֱ���ֵ�����Ͷ�ֵ�ź������� */
    xTaskCreate((TaskFunction_t )rs232_TASK,
                (const char*    )"rs232_TASK",
                (uint16_t       )RS232_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )RS232_TASK_PRIO,
                (TaskHandle_t*  )&RS232Task_Handler);								
		
		/* ͨ��SPI��ȡMAX6675оƬ�ɼ����ȵ�ż�¶����ݣ�ά����ȫ�ֱ��������Ͷ�ֵ�ź������� */
    xTaskCreate((TaskFunction_t )temprature_task,
                (const char*    )"temprature_task",
                (uint16_t       )Temprature_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Temprature_TASK_PRIO,
                (TaskHandle_t*  )&TempratureTask_Handler);	
								
    vTaskDelete(StartTask_Handler); /* ɾ����ʼ���� */
    taskEXIT_CRITICAL();            /* �˳��ٽ��� */
    
}

/**
 * @brief       lwIP��������
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void lwip_demo_task(void *pvParameters)
{
    pvParameters = pvParameters;
    
    uint8_t t = 0;
    while (1)
    {
        t ++;
        
        if ((t % 40) == 0)
        {
            LED0_TOGGLE();    /* ��תһ��LED0 */
        }
        vTaskDelay(5);
				
    }
	
}

/**
 * @brief       ϵͳ������
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void led_task(void *pvParameters)
{
    pvParameters = pvParameters;
    while (1)
    {
        LED1_TOGGLE();
        vTaskDelay(1000);
    }
}

/**
 * @brief       rs232��ʼ������3 ��ECU���Ͳ�ѯָ����յ��������ݺ�ά����ȫ�ֱ���
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void rs232_TASK(void *pvParameters)
{
    pvParameters = pvParameters;
    usart3_init(19200);//��ʼ��usart3 �����ʷ��������ҹ涨
		BaseType_t err = pdFALSE;
//		unsigned portBASE_TYPE uxHighWaterMark;
    while (1)
    {
        ECU_data_ask();
				//printf("ECU��ѯָ���\n");
        err=xSemaphoreTake(usart3Semaphore,100);//portMAX_DELAY
				if(err==pdTRUE)
				{
					printf("���յ�ECU��������\n");
					
					//taskENTER_CRITICAL();//�����ٽ���
					ECU_data_get();   //�Ӵ����ж�������װ��ȫ�ֱ���
					xSemaphoreGive(send_PC_Semaphore);	//���ʹ��ݸ���λ�����ź�
					//taskEXIT_CRITICAL();//�˳��ٽ���
				}
//				uxHighWaterMark=uxTaskGetStackHighWaterMark( RS232Task_Handler );	
//				printf("rs232_TASK size = %d\r\n",uxHighWaterMark);
				vTaskDelay(500);
    }
}



/**
 * @brief       ����ȫ�ֱ���ֵ ִ�м̵������գ��������
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void controller_relay_task(void *pvParameters)
{
	pvParameters = pvParameters;
	BaseType_t err = pdFALSE;
  																			//84M/168=0.5M�ļ���Ƶ�� �Զ���װ��Ϊ2000��PWMƵ��Ϊ0.5M/2000=250HZ   4ms���� 250Hz 1M
    TIM9_PWM_Init(2000-1,168-1,250-1);    	//84M/84=1M�ļ���Ƶ�ʣ��Զ���װ��Ϊ500����ôPWMƵ��Ϊ1M/500=2kHZ   

//		unsigned portBASE_TYPE uxHighWaterMark;
    while (1)
    {
			err=xSemaphoreTake(gpioSemaphore,portMAX_DELAY); //portMAX_DELAY
			if(err==pdTRUE)
			{
				printf("���ն�ֵ�ź���gpioSemaphore\n");
				printf("Engine_relay in task %d\n",battery_status);
        Engine_relay(battery_status);
				printf("Motor_relay in task %d\n",Electronic_Speed_Controller_relay);
				Motor_relay(Electronic_Speed_Controller_relay);
				printf("rectifier_relay in task %d\n",rectifier_relay);
				Rectifier_relay(rectifier_relay);
				printf("valve_servo in task %d\n",valve_servo);
				//TIM_SetTIM9Compare2(1750-valve_servo*10-1);
				TIM_SetTIM9Compare2(250+valve_servo*10-1);//���
			}
//			uxHighWaterMark=uxTaskGetStackHighWaterMark( Controller_relay_TASK_Handler );	
//			printf("controller_relay_task size = %d\r\n",uxHighWaterMark);
			vTaskDelay(500);
    }
}

/**
 * @brief       ͨ��SPI��ȡMAX6675оƬ�ɼ����ȵ�ż�¶����ݣ�ά����ȫ�ֱ��������Ͷ�ֵ�ź��� ���¶ȱ���
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void temprature_task(void *pvParameters)
{
		pvParameters = pvParameters;
		spi1_MAX6675_init();
		spi2_MAX6675_init();
		//unsigned portBASE_TYPE uxHighWaterMark;
    while (1)
    {
				
				first_cylinder_temprature = MAX6675_ReadTemprature_FILTER(&g_spi1_handler);
				second_cylinder_temprature = MAX6675_ReadTemprature_FILTER(&g_spi2_handler);
				//first_cylinder_temprature = MAX6675_ReadTemprature(&g_spi1_handler);
				//second_cylinder_temprature = MAX6675_ReadTemprature(&g_spi2_handler);//������� �����ܳ�ʼ����ʱ����ռĴ��� �������2000���¶�
				if(first_cylinder_temprature>210||second_cylinder_temprature>210)//�¶ȼ�ر�������
				{
					printf("�¶ȹ��ߣ�ϵͳ�Զ�����ͣ����������\n");
					Engine_relay(1);
				}
				xSemaphoreGive(send_PC_Temprature_Semaphore);	//���ʹ��ݸ���λ�����ź�
				printf("���Ͷ�ֵ�ź��������������ݷ��͸���λ����\n");
				//uxHighWaterMark=uxTaskGetStackHighWaterMark( TempratureTask_Handler );	
				//printf("temprature_task size = %d\r\n",uxHighWaterMark);
				vTaskDelay(500);//��ʽ��ʱ���СһЩ
    }
}



///*
//*********************************************************************************************************
//*    �� �� ��: vTaskMsgPro
//*    ����˵��: ��Ϣ����ʹ�ú�xQueueReceive��������vTaskTaskUserIF��Ϣ�����е����ݡ�
//*    ��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
//*    �� �� ֵ: ��
//*   �� �� ��: 3  
//*********************************************************************************************************
//*/
//static void vTaskMsgPro(void *pvParameters)
//{
//		pvParameters = pvParameters;
//    BaseType_t xResult;
//    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* �������ȴ�ʱ��Ϊ200ms */
//    MSG_R* ucQueueMsgValue;
//    
//    while(1)
//    {
//        xResult = xQueueReceive(xQueue2,                   /* ��Ϣ���о�� */
//                                (void *)&ucQueueMsgValue,  /* �洢���յ������ݵ�����ucQueueMsgValue�� */
//                                (TickType_t)xMaxBlockTime);/* ��������ʱ�� */
//        
//        if(xResult == pdPASS)
//        {
//            /* �ɹ����գ���ͨ�����ڽ����ݴ�ӡ���� */
//            printf("���յ���Ϣ��������ucQueueMsgValue = %d\r\n%d\r\n%d\r\n", ucQueueMsgValue->battery_status,
//																																						ucQueueMsgValue->Electronic_Speed_Controller_relay,
//																																						ucQueueMsgValue->rectifier_relay);
//        }
//        else
//        {
//            printf("û�н��յ���Ϣ��������");            
//        }
//    }
//}
