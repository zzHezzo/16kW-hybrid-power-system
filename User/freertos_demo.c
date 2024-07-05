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
/*变量信号量配置*/
//static QueueHandle_t xQueue2;
//extern volatile uint16_t valve_servo , battery_status , Electronic_Speed_Controller_relay, rectifier_relay;
//extern volatile uint16_t first_cylinder_temprature, second_cylinder_temprature,
//	engine_working_state, engine_speed, engine_warm_up, engine_idling_set,
//	/*		10					         12				     18				          24	*/
//	air_inflow, throttle_position;
//	/*	28				     74		*/

SemaphoreHandle_t gpioSemaphore;     //接收到上位机消息传递的信号量
SemaphoreHandle_t usart3Semaphore;	//接收到ECU消息传递的信号量		
SemaphoreHandle_t send_PC_Semaphore;	//发送给PC的信号量	
SemaphoreHandle_t send_PC_Temprature_Semaphore;	//发送给PC的温度信号量	
/******************************************************************************************************/



/*FreeRTOS配置*/
/* START_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define START_TASK_PRIO         5           /* 任务优先级 */
#define START_STK_SIZE          128         /* 任务堆栈大小 */
TaskHandle_t StartTask_Handler;             /* 任务句柄 */
void start_task(void *pvParameters);        /* 任务函数 */

/* LWIP_DEMO 任务 配置 和上位机通讯更新数组内容
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LWIP_DMEO_TASK_PRIO     11          /* 任务优先级 */
#define LWIP_DMEO_STK_SIZE      128        /* 任务堆栈大小 */
TaskHandle_t LWIP_Task_Handler;             /* 任务句柄 */
void lwip_demo_task(void *pvParameters);    /* 任务函数 */

/* LED_TASK 任务 配置 
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LED_TASK_PRIO           10          /* 任务优先级 */
#define LED_STK_SIZE            128         /* 任务堆栈大小 */
TaskHandle_t LEDTask_Handler;               /* 任务句柄 */
void led_task(void *pvParameters);          /* 任务函数 */

///* MsgPro_TASK 任务 配置
// * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
// */
//#define MsgPro_TASK_PRIO           3          /* 任务优先级 */
//#define MsgPro_STK_SIZE            128         /* 任务堆栈大小 */
//TaskHandle_t MsgProTask_Handler;               /* 任务句柄 */
//static void vTaskMsgPro(void *pvParameters);          /* 任务函数 */

/* Controller_relay_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define Controller_relay_TASK_PRIO        7          /* 任务优先级 */
#define Controller_relay_SIZE            128         /* 任务堆栈大小 */
TaskHandle_t Controller_relay_TASK_Handler;               /* 任务句柄 */
void controller_relay_task(void *pvParameters);          /* 任务函数 */

/* RS232_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define RS232_TASK_PRIO          8          /* 任务优先级 */
#define RS232_STK_SIZE          128         /* 任务堆栈大小 */
TaskHandle_t RS232Task_Handler;             /* 任务句柄 */
void rs232_TASK(void *pvParameters);        /* 任务函数 */

/* Temprature_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define Temprature_TASK_PRIO           9           /* 任务优先级 */
#define Temprature_STK_SIZE            256         /* 任务堆栈大小 */
TaskHandle_t TempratureTask_Handler;               /* 任务句柄 */
void temprature_task(void *pvParameters);          /* 任务函数 */
/******************************************************************************************************/


/**
 * @breif       加载UI
 * @param       mode :  bit0:0,不加载;1,加载前半部分UI
 *                      bit1:0,不加载;1,加载后半部分UI
 * @retval      无
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
            sprintf((char*)buf,"DHCP IP:%d.%d.%d.%d",g_lwipdev.ip[0],g_lwipdev.ip[1],g_lwipdev.ip[2],g_lwipdev.ip[3]);     /* 显示动态IP地址 */
        }
        else
        {
            sprintf((char*)buf,"Static IP:%d.%d.%d.%d",g_lwipdev.ip[0],g_lwipdev.ip[1],g_lwipdev.ip[2],g_lwipdev.ip[3]);    /* 打印静态IP地址 */
        }
        
        lcd_show_string(5, 130, 200, 16, 16, (char*)buf, MAGENTA);
        
        speed = ethernet_chip_get_speed();      /* 得到网速 */
        
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
 * @param       无
 * @retval      无
 */
void freertos_demo(void)
{
    /* start_task任务 */
    xTaskCreate((TaskFunction_t )start_task,
                (const char *   )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void *         )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t * )&StartTask_Handler);

    vTaskStartScheduler(); /* 开启任务调度 */
}

/**
 * @brief       start_task
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void start_task(void *pvParameters)
{
    pvParameters = pvParameters;
    
    g_lwipdev.lwip_display_fn = lwip_test_ui;
    
    g_lwipdev.lwip_display_fn = lwip_test_ui;
    
    lwip_test_ui(1);    /* 加载后前部分UI */
    
    while (lwip_comm_init() != 0)
    {
        lcd_show_string(30, 110, 200, 16, 16, "lwIP Init failed!!", RED);
        delay_ms(500);
        lcd_fill(30, 50, 200 + 30, 50 + 16, WHITE);
        lcd_show_string(30, 110, 200, 16, 16, "Retrying...       ", RED);
        delay_ms(500);
        LED1_TOGGLE();
    }
    
    while (!ethernet_read_phy(PHY_SR))  /* 检查MCU与PHY芯片是否通信成功 */
    {
        printf("MCU与PHY芯片通信失败，请检查电路或者源码！！！！\r\n");
    }
    
    while ((g_lwipdev.dhcpstatus != 2)&&(g_lwipdev.dhcpstatus != 0XFF))  /* 等待DHCP获取成功/超时溢出 */
    {
        vTaskDelay(5);
    }
		
    /*创建二值信号量*/
		usart3Semaphore = xSemaphoreCreateBinary();
		gpioSemaphore = xSemaphoreCreateBinary();
		send_PC_Semaphore = xSemaphoreCreateBinary();
		send_PC_Temprature_Semaphore = xSemaphoreCreateBinary();
    
    taskENTER_CRITICAL();           /* 进入临界区 */
		tcpecho_init();/*作为服务器监听*/
    /* 创建lwIP任务 */
    xTaskCreate((TaskFunction_t )lwip_demo_task,
                (const char*    )"lwip_demo_task",
                (uint16_t       )LWIP_DMEO_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LWIP_DMEO_TASK_PRIO,
                (TaskHandle_t*  )&LWIP_Task_Handler);

    /* LED测试任务 */
    xTaskCreate((TaskFunction_t )led_task,
                (const char*    )"led_task",
                (uint16_t       )LED_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )LED_TASK_PRIO,
                (TaskHandle_t*  )&LEDTask_Handler);
    
//		/* MsgPro_TASK任务 */
//    xTaskCreate((TaskFunction_t )vTaskMsgPro,
//                (const char*    )"vTaskMsgPro",
//                (uint16_t       )MsgPro_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )MsgPro_TASK_PRIO,
//                (TaskHandle_t*  )&MsgProTask_Handler);
								
		/* 继电器根据指令开关任务 */
    xTaskCreate((TaskFunction_t )controller_relay_task,
                (const char*    )"controller_relay_task",
                (uint16_t       )Controller_relay_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Controller_relay_TASK_PRIO,
                (TaskHandle_t*  )&Controller_relay_TASK_Handler);
								
		/* RS232发送查询ECU指令和根据ECU接收回来的指令维护全局变量值并发送二值信号量任务 */
    xTaskCreate((TaskFunction_t )rs232_TASK,
                (const char*    )"rs232_TASK",
                (uint16_t       )RS232_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )RS232_TASK_PRIO,
                (TaskHandle_t*  )&RS232Task_Handler);								
		
		/* 通过SPI读取MAX6675芯片采集的热电偶温度数据，维护进全局变量并发送二值信号量任务 */
    xTaskCreate((TaskFunction_t )temprature_task,
                (const char*    )"temprature_task",
                (uint16_t       )Temprature_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Temprature_TASK_PRIO,
                (TaskHandle_t*  )&TempratureTask_Handler);	
								
    vTaskDelete(StartTask_Handler); /* 删除开始任务 */
    taskEXIT_CRITICAL();            /* 退出临界区 */
    
}

/**
 * @brief       lwIP运行例程
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
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
            LED0_TOGGLE();    /* 翻转一次LED0 */
        }
        vTaskDelay(5);
				
    }
	
}

/**
 * @brief       系统在运行
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
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
 * @brief       rs232初始化串口3 向ECU发送查询指令，接收到返回数据后维护进全局变量
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void rs232_TASK(void *pvParameters)
{
    pvParameters = pvParameters;
    usart3_init(19200);//初始化usart3 波特率发动机厂家规定
		BaseType_t err = pdFALSE;
//		unsigned portBASE_TYPE uxHighWaterMark;
    while (1)
    {
        ECU_data_ask();
				//printf("ECU查询指令发送\n");
        err=xSemaphoreTake(usart3Semaphore,100);//portMAX_DELAY
				if(err==pdTRUE)
				{
					printf("接收到ECU返回数据\n");
					
					//taskENTER_CRITICAL();//进入临界区
					ECU_data_get();   //从串口中读数并封装进全局变量
					xSemaphoreGive(send_PC_Semaphore);	//发送传递给上位机的信号
					//taskEXIT_CRITICAL();//退出临界区
				}
//				uxHighWaterMark=uxTaskGetStackHighWaterMark( RS232Task_Handler );	
//				printf("rs232_TASK size = %d\r\n",uxHighWaterMark);
				vTaskDelay(500);
    }
}



/**
 * @brief       根据全局变量值 执行继电器开闭，舵机控制
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void controller_relay_task(void *pvParameters)
{
	pvParameters = pvParameters;
	BaseType_t err = pdFALSE;
  																			//84M/168=0.5M的计数频率 自动重装载为2000，PWM频率为0.5M/2000=250HZ   4ms周期 250Hz 1M
    TIM9_PWM_Init(2000-1,168-1,250-1);    	//84M/84=1M的计数频率，自动重装载为500，那么PWM频率为1M/500=2kHZ   

//		unsigned portBASE_TYPE uxHighWaterMark;
    while (1)
    {
			err=xSemaphoreTake(gpioSemaphore,portMAX_DELAY); //portMAX_DELAY
			if(err==pdTRUE)
			{
				printf("接收二值信号量gpioSemaphore\n");
				printf("Engine_relay in task %d\n",battery_status);
        Engine_relay(battery_status);
				printf("Motor_relay in task %d\n",Electronic_Speed_Controller_relay);
				Motor_relay(Electronic_Speed_Controller_relay);
				printf("rectifier_relay in task %d\n",rectifier_relay);
				Rectifier_relay(rectifier_relay);
				printf("valve_servo in task %d\n",valve_servo);
				//TIM_SetTIM9Compare2(1750-valve_servo*10-1);
				TIM_SetTIM9Compare2(250+valve_servo*10-1);//舵机
			}
//			uxHighWaterMark=uxTaskGetStackHighWaterMark( Controller_relay_TASK_Handler );	
//			printf("controller_relay_task size = %d\r\n",uxHighWaterMark);
			vTaskDelay(500);
    }
}

/**
 * @brief       通过SPI读取MAX6675芯片采集的热电偶温度数据，维护进全局变量并发送二值信号量 含温度报警
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
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
				//second_cylinder_temprature = MAX6675_ReadTemprature(&g_spi2_handler);//这个正常 但不能初始化的时候清空寄存器 否则读个2000多温度
				if(first_cylinder_temprature>210||second_cylinder_temprature>210)//温度监控报警功能
				{
					printf("温度过高，系统自动报警停车！！！！\n");
					Engine_relay(1);
				}
				xSemaphoreGive(send_PC_Temprature_Semaphore);	//发送传递给上位机的信号
				printf("发送二值信号量，即将将数据发送给上位机！\n");
				//uxHighWaterMark=uxTaskGetStackHighWaterMark( TempratureTask_Handler );	
				//printf("temprature_task size = %d\r\n",uxHighWaterMark);
				vTaskDelay(500);//正式的时候改小一些
    }
}



///*
//*********************************************************************************************************
//*    函 数 名: vTaskMsgPro
//*    功能说明: 消息处理，使用函xQueueReceive接收任务vTaskTaskUserIF消息队列中的数据。
//*    形    参: pvParameters 是在创建该任务时传递的形参
//*    返 回 值: 无
//*   优 先 级: 3  
//*********************************************************************************************************
//*/
//static void vTaskMsgPro(void *pvParameters)
//{
//		pvParameters = pvParameters;
//    BaseType_t xResult;
//    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200); /* 设置最大等待时间为200ms */
//    MSG_R* ucQueueMsgValue;
//    
//    while(1)
//    {
//        xResult = xQueueReceive(xQueue2,                   /* 消息队列句柄 */
//                                (void *)&ucQueueMsgValue,  /* 存储接收到的数据到变量ucQueueMsgValue中 */
//                                (TickType_t)xMaxBlockTime);/* 设置阻塞时间 */
//        
//        if(xResult == pdPASS)
//        {
//            /* 成功接收，并通过串口将数据打印出来 */
//            printf("接收到消息队列数据ucQueueMsgValue = %d\r\n%d\r\n%d\r\n", ucQueueMsgValue->battery_status,
//																																						ucQueueMsgValue->Electronic_Speed_Controller_relay,
//																																						ucQueueMsgValue->rectifier_relay);
//        }
//        else
//        {
//            printf("没有接收到消息队列数据");            
//        }
//    }
//}
